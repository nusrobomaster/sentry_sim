import rclpy
import threading
from enum import Enum

from robot_navigator import BasicNavigator, NavigationResult

import time
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import UInt8, UInt16MultiArray, Bool, Float32, String, Int8
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
from rclpy.duration import Duration 
from threading import Timer

# Enums
class State(Enum):
    InitIdleState = 0
    SurveillingState = 1
    NavigatingState = 2
    ShootingState = 3
    AntiPushState = 4

class Status(Enum):
    InitIdleStatus = 0
    LowHealthStatus = 1
    CombatStatus = 2

class BehaviorMode(Enum): # Left Trigger
    UndefinedMode = 0
    SeriousMode = 1    # Competition
    CasualMode = 3      # Idle
    IdleMode = 2   # Testing


class RobotNode(Node):

    def __init__(self):
        super().__init__('sentry_fsm') 

        self.get_logger().info('Sentry FSM Node Started')

        # State Publisher
        self.state_pub = self.create_publisher(String, '/sentry/fsm_state', 10)
        
        # Referee Override
        self.referee_override_sub = self.create_subscription(
            Int8, '/referee/override', self.referee_override_callback, 10)

        self.left_trigger_subscriber = self.create_subscription(
            UInt8, 'sen/remote_left_trigger', self.left_trigger_callback, 10)

        self.competition_status_sub = self.create_subscription(
            UInt16MultiArray, 'sen/competition_status', self.competition_status_callback, 10)
        
        self.occupation_status_sub = self.create_subscription(
            UInt16MultiArray, 'sen/occupation_status', self.occupation_status_callback, 10)
        
        self.target_detected_sub = self.create_subscription(
            Bool, '/cv_detected', self.detected_opponent_callback, 10)

        self.cur_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.cur_pose_callback, 10)
        
        # target_dist sub
        self.target_dist_sub = self.create_subscription(
            Float32, '/cv_dist', self.target_dist_callback, 10)
        
        self.in_supply_sub = self.create_subscription(
            Bool, 'sen/in_supply', self.in_supply_callback, 10)
        
        self.in_central_sub = self.create_subscription(
            Bool, 'sen/in_central', self.in_central_callback, 10)

        self.spin_publisher_ = self.create_publisher(Bool, 'sen/chassis_spin_cmd', 10)
        self.shoot_publisher_ = self.create_publisher(Bool,'sen/shoot_cmd', 10)
        self.surveil_publisher_ = self.create_publisher(Bool,'sen/surveil_cmd', 10)
        self.aim_publisher_ = self.create_publisher(Bool, 'sen/aim_cmd', 10)
        self.navigating_publisher_ = self.create_publisher(Bool, 'sen/is_navigating', 10)

        ################################## TUNABLE CONSTANTS ###############################
        self.GAP = 0.5
        self.LOWHP = 100
        self.MATCH_TIME = 300
        self.STANDBY_TIME = 0

        # DEFENSE ZONE COORDINATES
        self.ZONE_X_MIN = 5.0
        self.ZONE_X_MAX = 8.5
        self.ZONE_Y_MIN = 3.0
        self.ZONE_Y_MAX = 6.0

        # ROUTE TO CENTRAL (Waypoint Array)
        # !!! REMINDER: Update these coordinates
        # The last dictionary in the list MUST be the actual Central Zone.
        self.CENTRAL_ROUTE = [
            {'x': 1.50, 'y': 1.50, 'z_ori': 0.0, 'w_ori': 1.0},   # Waypoint 1 (Near Spawn)
            {'x': 3.00, 'y': 2.50, 'z_ori': 0.0, 'w_ori': 1.0},   # Waypoint 2
            {'x': 4.50, 'y': 3.50, 'z_ori': 0.0, 'w_ori': 1.0},   # Waypoint 3
            {'x': 5.50, 'y': 4.00, 'z_ori': 0.0, 'w_ori': 1.0},   # Waypoint 4
            {'x': 6.32, 'y': 4.19, 'z_ori': -0.707, 'w_ori': 0.707} # Final Destination (Central Zone)
        ]
        self.current_route_idx = 0  # Tracks which waypoint we are navigating to

        self.SUPPLY_ZONE_X = 11.16
        self.SUPPLY_ZONE_Y = 1.08
        self.SUPPLY_ZONE_Z = 0.0 
        self.SUPPLY_ORI_Z = 0.707
        self.SUPPLY_ZONE_W = 0.707

        self.costmap_cleanup_interval = 5.0  # Time in seconds
        self.timer = None

        self.is_navigating = 0
        # Navigation timeout tracking
        self.nav_start_time = None
        self.nav_timeout = 120.0 # Seconds (increased temporarily)
        
        # Referee system data
        self.game_progress = 0
        self.time_left = 0
        self.robot_id = 0
        self.current_hp = 500 # Default to alive
        self.red_hero_hp = 0
        self.red_standard_hp = 0
        self.red_sentry_hp = 0
        self.blue_hero_hp = 0
        self.blue_standard_hp = 0
        self.blue_sentry_hp = 0
        self.in_supply = False
        self.in_central = False

        self.opponent_detected = 0
        self.status_transition = 0 
        self.target_dist = 99.9
        
        self.current_state = State.InitIdleState
        self.current_status = Status.InitIdleStatus
        self.prev_status = Status.InitIdleStatus
        self.current_mode = BehaviorMode.IdleMode
        self.cur_pose = PoseStamped()
        self.cur_pose.header.frame_id = 'map'
        self.target_dist = 0
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.nav_goal_msg = PoseStamped()
        self.nav_goal_msg.header.frame_id = 'map'

        self.has_reached_central = False

        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup() 
        
        self.navigator.waitUntilNav2Active()

        self.navigator.clearAllCostmaps()

        self.clear_costmap_counter = 0

        self.behavior_thread = threading.Thread(target=self.behavior_loop)
        self.stop_thread = threading.Event()
        self.behavior_thread.start()
        self.init_clean_costmaps()

        self.sentry_ready = False
        self.get_logger().info('FSM Initialized - Waiting for Game Start')

    def __del__(self):
        self.stop_thread.set()
        if self.behavior_thread.is_alive():
            self.behavior_thread.join()
    
    # Callback for Manual Override
    def referee_override_callback(self, msg):
        self.get_logger().info(f'OVERRIDE: Setting game_progress to {msg.data}')
        self.game_progress = msg.data
        if self.game_progress == 4: # GAME_RUNNING
            self.current_mode = BehaviorMode.SeriousMode 
            self.time_left = 300 
            self.current_hp = 500
        elif self.game_progress == 0:
            self.current_mode = BehaviorMode.IdleMode
            self.has_reached_central = False # Reset on stop

    def init_clean_costmaps(self):
        self.timer = Timer(self.costmap_cleanup_interval, self.navigator.clearAllCostmaps)
        # self.timer.start()

    def dist_xy_decomp(self, dist):
        quat_x = self.cur_pose.pose.orientation.x
        quat_y = self.cur_pose.pose.orientation.y
        quat_z = self.cur_pose.pose.orientation.z
        quat_w = self.cur_pose.pose.orientation.w
        yaw = math.atan2(2*(quat_w*quat_z + quat_x*quat_y), 
                     1 - 2*(quat_y*quat_y + quat_z*quat_z))
        
        dist_x = dist * math.cos(yaw)
        dist_y = dist * math.sin(yaw)
        
        return dist_x, dist_y

    def left_trigger_callback(self, msg):
        self.current_mode = BehaviorMode(msg.data)
        
    def competition_status_callback(self, msg):
        self.game_progress = msg.data[0]
        self.time_left = msg.data[1]
        self.robot_id = msg.data[2]
        self.current_hp = msg.data[3]
        self.red_hero_hp = msg.data[4]
        self.red_standard_hp = msg.data[5]
        self.red_sentry_hp = msg.data[6]
        self.blue_hero_hp = msg.data[7]
        self.blue_standard_hp = msg.data[8]
        self.blue_sentry_hp = msg.data[9]

    def occupation_status_callback(self, msg):
        self.in_supply = msg.data[0]
        self.in_central = msg.data[1]

    def cur_pose_callback(self, msg):
        # self.cur_pose = msg.data
        self.cur_pose = PoseStamped()
        self.cur_pose.header = msg.header
        self.cur_pose.pose = msg.pose.pose
        
    def detected_opponent_callback(self, msg):
        self.opponent_detected = msg.data

    def target_dist_callback(self, msg):
        self.target_dist = msg.data
        if self.target_dist == 0.0:
            self.target_dist = 99.9

    def in_supply_callback(self, msg):
        self.in_supply = msg.data
        if self.in_supply:
            self.has_reached_central = False # Left zone to heal
            self.current_route_idx = 0  # Reset route when back at supply

    def in_central_callback(self, msg):
        self.in_central = msg.data
        if self.in_central:
            self.has_reached_central = True # Reached zone
            self.current_route_idx = len(self.CENTRAL_ROUTE) - 1  # Lock to final waypoint

    def is_in_defense_zone(self):
        x = self.cur_pose.pose.position.x
        y = self.cur_pose.pose.position.y
        if x == 0.0 and y == 0.0: return True
        return (self.ZONE_X_MIN <= x <= self.ZONE_X_MAX) and \
               (self.ZONE_Y_MIN <= y <= self.ZONE_Y_MAX)

    # HELPER FOR FORCE RETURNING
    def set_goal_central(self):
        # Helper specifically for Anti-Push to force it back to the FINAL destination
        final_wp = self.CENTRAL_ROUTE[-1]
        self.nav_goal_msg.header.stamp = self.navigator.get_clock().now().to_msg()
        self.nav_goal_msg.pose.position.x = final_wp['x']
        self.nav_goal_msg.pose.position.y = final_wp['y']
        self.nav_goal_msg.pose.position.z = 0.0
        self.nav_goal_msg.pose.orientation.z = final_wp['z_ori']
        self.nav_goal_msg.pose.orientation.w = final_wp['w_ori']
        self.send_nav_goal()

    def behavior_loop(self):
        rate = self.create_rate(10)
        while rclpy.ok() and not self.stop_thread.is_set():
            
            # Publish State (Debug)
            state_msg = String()
            state_msg.data = f"State: {self.current_state.name} | WP: {self.current_route_idx} | HP: {self.current_hp} | InZone: {self.is_in_defense_zone()}"
            self.state_pub.publish(state_msg)

            # Publish System Commands
            self.send_spin_cond()
            self.send_aim_cond()
            self.send_surveil_cond()
            self.send_nav_cond()
            self.send_shoot_cond()

            # Update Status
            self.prev_status = self.current_status

            if self.game_progress != 4 or self.time_left > (self.MATCH_TIME - self.STANDBY_TIME) or self.current_mode == BehaviorMode.IdleMode:
                self.current_status = Status.InitIdleStatus
            elif self.current_hp <= self.LOWHP:
                self.current_status = Status.LowHealthStatus
            else:
                self.current_status = Status.CombatStatus
                        
            # PRIORITY 1: LOW HP
            if self.current_status == Status.LowHealthStatus:
                 if self.current_state != State.NavigatingState and not self.in_supply:
                     self.get_logger().info('CRITICAL: Low HP! Retreating to Supply.')
                     self.navigator.cancelNav()
                     self.set_goal() # Uses Status to pick Supply
                     self.current_state = State.NavigatingState

            # PRIORITY 2: ANTI-PUSH
            elif self.current_status == Status.CombatStatus and \
                 self.has_reached_central and \
                 not self.is_in_defense_zone():
                
                if self.current_state != State.AntiPushState:
                    self.get_logger().warn('ZONE BREACHED! Engaging Anti-Push.')
                    self.navigator.cancelNav()
                    self.set_goal_central()
                    self.current_state = State.AntiPushState

            # INTERRUPT: ENEMY SPOTTED
            elif self.opponent_detected and self.current_status != Status.InitIdleStatus:
                if self.current_state != State.AntiPushState:
                    if self.current_state == State.NavigatingState:
                        self.get_logger().warn('INTERRUPT: Enemy Spotted! ABORTING NAVIGATION.')
                        self.navigator.cancelNav()
                    self.current_state = State.ShootingState

            # INTERRUPT: STATUS CHANGED
            elif self.current_state == State.NavigatingState and self.current_status != self.prev_status:
                self.get_logger().warn('INTERRUPT: Status Changed! ABORTING NAV to Reroute.')
                self.navigator.cancelNav()
                self.current_state = State.SurveillingState

            if self.current_state != State.NavigatingState and \
               self.current_state != State.ShootingState and \
               self.current_state != State.AntiPushState:
                
                # LOW HP -> GO SUPPLY
                if self.current_status == Status.LowHealthStatus:
                    if not self.in_supply:
                        self.get_logger().info('Low Health -> Routing to Supply')
                        self.set_goal()
                        self.current_state = State.NavigatingState
                
                # COMBAT -> GO CENTRAL
                elif self.current_status == Status.CombatStatus and self.current_mode == BehaviorMode.SeriousMode:
                    if not self.in_central and not self.has_reached_central:
                        self.get_logger().info(f'Resuming Patrol -> Routing to WP {self.current_route_idx}')
                        self.set_goal()
                        self.current_state = State.NavigatingState

            # STATE EXECUTION
            if self.current_state == State.InitIdleState:
                if not self.navigator.isNavComplete():
                    self.navigator.cancelNav()

            elif self.current_state == State.SurveillingState:
                pass

            elif self.current_state == State.NavigatingState:
                # Check for Timeout
                if self.nav_start_time is not None:
                    if (time.time() - self.nav_start_time) > self.nav_timeout:
                        self.get_logger().warn('Navigation Timeout! Cancelling.')
                        self.navigator.cancelNav()
                        self.current_state = State.SurveillingState
                
                # Check for Completion
                if self.navigator.isNavComplete():
                    result = self.navigator.getResult()
                    if result == NavigationResult.SUCCEEDED:
                        
                        if self.current_status == Status.LowHealthStatus:
                             self.get_logger().info("Arrived at Supply.")
                             self.in_supply = True
                             self.has_reached_central = False
                             self.current_state = State.SurveillingState
                        else:
                             # Check if we have more waypoints to go
                             if self.current_route_idx < len(self.CENTRAL_ROUTE) - 1:
                                 self.current_route_idx += 1
                                 self.get_logger().info(f"Waypoint Reached. Moving to WP {self.current_route_idx}")
                                 self.set_goal()  # Send next waypoint immediately
                             else:
                                 self.get_logger().info("Arrived at Central Zone.")
                                 self.in_central = True
                                 self.has_reached_central = True
                                 self.current_state = State.SurveillingState
                    else:
                        self.current_state = State.SurveillingState

            elif self.current_state == State.ShootingState:
                # If enemy disappears, go back to patrolling
                if not self.opponent_detected:
                    self.get_logger().info("Enemy lost. Resuming surveillance.")
                    self.current_state = State.SurveillingState

            elif self.current_state == State.AntiPushState:
                # Check if we recovered the zone
                if self.is_in_defense_zone():
                    self.get_logger().info("Zone Recovered. Resuming Defense.")
                    self.navigator.cancelNav()
                    # If enemy still there -> Shoot, else -> Surveil
                    if self.opponent_detected:
                        self.current_state = State.ShootingState
                    else:
                        self.current_state = State.SurveillingState
                
                # Retry logic if stuck
                if self.navigator.isNavComplete():
                    if not self.is_in_defense_zone():
                        self.get_logger().warn("Anti-Push stuck. Retrying...")
                        self.set_goal_central()

            rate.sleep()

    def set_goal(self):
        if self.current_status == Status.LowHealthStatus:
            # Set supply zone goal
            self.nav_goal_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            self.nav_goal_msg.pose.position.x = self.SUPPLY_ZONE_X
            self.nav_goal_msg.pose.position.y = self.SUPPLY_ZONE_Y
            self.nav_goal_msg.pose.position.z = self.SUPPLY_ZONE_Z
            self.nav_goal_msg.pose.orientation.z = self.SUPPLY_ORI_Z
            self.nav_goal_msg.pose.orientation.w = self.SUPPLY_ZONE_W
        else:
            wp = self.CENTRAL_ROUTE[self.current_route_idx]
            self.nav_goal_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            self.nav_goal_msg.pose.position.x = wp['x']
            self.nav_goal_msg.pose.position.y = wp['y']
            self.nav_goal_msg.pose.position.z = 0.0
            self.nav_goal_msg.pose.orientation.z = wp['z_ori']
            self.nav_goal_msg.pose.orientation.w = wp['w_ori']

        if self.navigator.isNavComplete():
            self.nav_start_time = time.time()
            self.send_nav_goal()

    def send_nav_goal(self):
        self.send_goal_future = self.navigator.goToPose(self.nav_goal_msg)
        self.prev_state = self.current_state
        self.is_navigating = 1
        self.get_logger().info('Nav Goal Sent')

    def send_spin_cond(self):
        spin_bool = Bool()
        spin_bool.data = (self.current_state == State.SurveillingState) or (self.current_state == State.ShootingState)
        self.spin_publisher_.publish(spin_bool)

    def send_aim_cond(self):
        aim_bool = Bool()
        aim_bool.data = (self.current_state == State.SurveillingState) or \
                        (self.current_state == State.ShootingState) or \
                        (self.current_state == State.AntiPushState)
        self.aim_publisher_.publish(aim_bool)

    def send_surveil_cond(self):
        surveil_bool = Bool()
        surveil_bool.data = (self.current_state == State.SurveillingState)
        self.surveil_publisher_.publish(surveil_bool)

    def send_nav_cond(self):
        nav_bool = Bool()
        nav_bool.data = (self.current_state == State.NavigatingState) or \
                        (self.current_state == State.AntiPushState)
        self.navigating_publisher_.publish(nav_bool)
        
    def send_shoot_cond(self):
        shoot_bool = Bool()
        fire = (self.current_state == State.ShootingState) or \
               (self.current_state == State.AntiPushState and self.opponent_detected)
        shoot_bool.data = fire
        self.shoot_publisher_.publish(shoot_bool)

    def get_result_callback(self, result):
        self.nav_start_time = None

        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info("Successfully navigated to pose")
            self.get_logger().info("Start spinning")
            self.is_navigating = 0 
            self.dur_nav = 0

        elif result == NavigationResult.CANCELED:
            self.is_navigating = 0 
            self.dur_nav = 0 
            self.get_logger().info("Navigation to pose canceled")
        elif result == NavigationResult.FAILED:
            self.is_navigating = 0
            self.dur_nav = 0

def main(args=None):
    rclpy.init()
    print("Starting Sentry FSM")
    node = RobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.navigator)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        node.navigator.destroy_node()
        if node.timer:
            node.timer.cancel()
        rclpy.shutdown()
 
if __name__ == '__main__':
  main()