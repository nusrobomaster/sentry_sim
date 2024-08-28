#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Path2WaypointNode : public rclcpp::Node
{
public:
    Path2WaypointNode()
    : Node("path2waypoint"),
      odom_topic_("state_estimation"),
      path_topic_("move_base1/NavfnROS/plan"),
      distance_thre_(1.2),
      distance_tole_(0.6),
      vehicleX_(0.0),
      vehicleY_(0.0),
      vehicleZ_(0.0),
      curTime_(0.0),
      waypointTime_(0.0),
      way_point_temp_x_(0.0),
      way_point_temp_y_(0.0),
      Flag_get_new_path_(false),
      Flag_finish_path_(false),
      Flag_switch_goal_(false)
    {
        this->declare_parameter<std::string>("odom_topic", odom_topic_);
        this->declare_parameter<std::string>("path_topic", path_topic_);
        this->declare_parameter<float>("distance_thre", distance_thre_);

        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("path_topic", path_topic_);
        this->get_parameter("distance_thre", distance_thre_);

        subPose_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 5, std::bind(&Path2WaypointNode::poseHandler, this, std::placeholders::_1));

        subPath_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 5, std::bind(&Path2WaypointNode::pathHandler, this, std::placeholders::_1));

        pubWaypoint_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
        
        geometry_msgs::msg::PointStamped waypointMsgs;
        waypointMsgs_.header.frame_id = "map";
        rate_ = std::make_shared<rclcpp::Rate>(100);
    }

    void spin()
    {
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());

            if (!way_point_array_.empty())
            {
                if (Flag_get_new_path_)
                {
                    goal_point_ = way_point_array_.front();
                    path_index_ = 0;
                    Flag_switch_goal_ = true;
                    Flag_get_new_path_ = false;
                }
                else
                {
                    double goal_point_distance = sqrt(pow(goal_point_.pose.position.x - vehicleX_, 2) 
                                                    + pow(goal_point_.pose.position.y - vehicleY_, 2));
                    
                    if ((goal_point_distance < distance_tole_) && (path_index_ < way_point_array_.size() - 1))
                    {
                        path_index_++;
                        goal_point_ = way_point_array_.at(path_index_);
                        Flag_switch_goal_ = true;
                    }
                    else if (path_index_ == way_point_array_.size() - 1)
                    {
                        Flag_finish_path_ = true;
                    }
                }
            }

            if (Flag_switch_goal_)
            {
                waypointMsgs_.header.stamp = this->get_clock()->now();
                waypointMsgs_.point.x = goal_point_.pose.position.x;
                waypointMsgs_.point.y = goal_point_.pose.position.y;
                waypointMsgs_.point.z = 0;
                pubWaypoint_->publish(waypointMsgs_);
            }

            rate_->sleep();
        }
    }

private:
    void poseHandler(const nav_msgs::msg::Odometry::SharedPtr pose)
    {
        curTime_ = pose->header.stamp.sec;

        vehicleX_ = pose->pose.pose.position.x;
        vehicleY_ = pose->pose.pose.position.y;
        vehicleZ_ = pose->pose.pose.position.z;
    }

    void pathHandler(const nav_msgs::msg::Path::SharedPtr path)
    {
        /* Obtains the path and puts it in the container to traverse
           Everytime chooses an output that is 1m away
           If no point exists that is 1m away after finish traversing, outputs final point
        */
        if (!path->poses.empty()) {
            Flag_get_new_path_ = true;
            way_point_array_.clear();
            std::vector<geometry_msgs::msg::PoseStamped> path_array = path->poses;
            geometry_msgs::msg::PoseStamped first_point = *path_array.begin();
            geometry_msgs::msg::PoseStamped last_point = *(path_array.end() - 1);
            float distance_2d = 0;
            double temp_x = vehicleX_;
            double temp_y = vehicleY_;

            for (const auto &pose_stamped : path_array)
            {
                distance_2d = sqrt(pow(pose_stamped.pose.position.x - temp_x, 2) + 
                                    pow(pose_stamped.pose.position.y - temp_y, 2));

                if (distance_2d > distance_thre_) {
                    way_point_array_.push_back(pose_stamped);
                    temp_x = pose_stamped.pose.position.x;
                    temp_y = pose_stamped.pose.position.y;
                    RCLCPP_INFO(this->get_logger(), "Chosen point: (%f, %f)", temp_x, temp_y);
                }
            }
            way_point_array_.push_back(last_point);
        }
    }

    std::string odom_topic_, path_topic_;
    float distance_thre_, distance_tole_;
    float vehicleX_, vehicleY_, vehicleZ_;
    double curTime_, waypointTime_, way_point_temp_x_, way_point_temp_y_;
    bool Flag_get_new_path_, Flag_finish_path_, Flag_switch_goal_;

    std::vector<geometry_msgs::msg::PoseStamped> way_point_array_;
    geometry_msgs::msg::PoseStamped goal_point_;
    // geometry_msgs::msg::PointStamped waypointMsgs_;
    int path_index_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subPose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubWaypoint_;
    std::shared_ptr<rclcpp::Rate> rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Path2WaypointNode>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
