import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Int8, Bool

class SentryRemote(Node):
    def __init__(self):
        super().__init__('sentry_mvp_remote')
        self.status_pub = self.create_publisher(UInt16MultiArray, 'sen/competition_status', 10)
        self.override_pub = self.create_publisher(Int8, '/referee/override', 10)
        self.enemy_pub = self.create_publisher(Bool, '/cv_detected', 10)

        print("\n=== CONTROLS ===")
        print("[1] START MATCH  (Full HP -> Go Central)")
        print("[2] RETREAT      (Low HP  -> Go Supply)")
        print("[3] SHOOT ENEMY  (Stop & PEW PEW)")
        print("[4] CLEAR ENEMY  (Resume Patrol)")

        self.run_loop()

    def run_loop(self):
        while True:
            cmd = input("\nCommand > ")
            if cmd == '1':
                self.override_pub.publish(Int8(data=4))
                msg = UInt16MultiArray()
                msg.data = [4, 300, 7, 500, 0, 0, 0, 0, 0, 0] # HP = 500
                self.status_pub.publish(msg)
                print(">> GO FIGHT")
            elif cmd == '2':
                msg = UInt16MultiArray()
                msg.data = [4, 300, 7, 50, 0, 0, 0, 0, 0, 0] # HP = 50
                self.status_pub.publish(msg)
                print(">> RETREAT")
            elif cmd == '3':
                self.enemy_pub.publish(Bool(data=True))
                print(">> SHOOTING")
            elif cmd == '4':
                self.enemy_pub.publish(Bool(data=False))
                print(">> RESUMING")

def main():
    rclpy.init()
    SentryRemote()

if __name__ == '__main__':
    main()