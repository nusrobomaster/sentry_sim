#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_ros/transforms.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

/*
The main function of this code is the control the movement of the chassis and broadcast related messages 
Subscribes to:
  Lidar pointcloud
  Terrain pointcloud
  speed topic
Publishes:
  registered scan
  odometry information
  tf transformation from map to sensor
*/

using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double cameraOffsetZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

geometry_msgs::msg::Quaternion Odom_quat_get;
geometry_msgs::msg::Point Odom_pose_get;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

std::vector<int> scanInd;

rclcpp::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubScanPointer = nullptr;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubRobotSpeed = nullptr;

std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
// std::shared_ptr<tf2_ros::Buffer> tf_buffer;
// std::shared_ptr<tf2_ros::TransformListener> tf_listener;

class RobotSimulator : public rclcpp::Node
{
public:
  RobotSimulator() : Node("robot_simulator")
  {
    tf_buffer_ = make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Declare and get parameters
    this->declare_parameter<bool>("use_gazebo_time", use_gazebo_time);
    this->declare_parameter<double>("cameraOffsetZ", cameraOffsetZ);
    this->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
    this->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
    this->declare_parameter<double>("vehicleHeight", vehicleHeight);
    this->declare_parameter<double>("vehicleX", vehicleX);
    this->declare_parameter<double>("vehicleY", vehicleY);
    this->declare_parameter<double>("vehicleZ", vehicleZ);
    this->declare_parameter<double>("terrainZ", terrainZ);
    this->declare_parameter<double>("vehicleYaw", vehicleYaw);
    this->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
    this->declare_parameter<double>("groundHeightThre", groundHeightThre);
    this->declare_parameter<bool>("adjustZ", adjustZ);
    this->declare_parameter<double>("terrainRadiusZ", terrainRadiusZ);
    this->declare_parameter<int>("minTerrainPointNumZ", minTerrainPointNumZ);
    this->declare_parameter<bool>("adjustIncl", adjustIncl);
    this->declare_parameter<double>("terrainRadiusIncl", terrainRadiusIncl);
    this->declare_parameter<int>("minTerrainPointNumIncl", minTerrainPointNumIncl);
    this->declare_parameter<double>("InclFittingThre", InclFittingThre);
    this->declare_parameter<double>("maxIncl", maxIncl);

    // Subscribers
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2, std::bind(&RobotSimulator::scanHandler, this, std::placeholders::_1));
    sub_terrain_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 2, std::bind(&RobotSimulator::terrainCloudHandler, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/A/car0/odom", 100, std::bind(&RobotSimulator::odomHandler, this, std::placeholders::_1));
    sub_speed_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5, std::bind(&RobotSimulator::speedHandler, this, std::placeholders::_1));

    // Publishers
    pub_vehicle_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);
    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 2);
    pub_speed_ = this->create_publisher<geometry_msgs::msg::Twist>("/A/car0/cmd_vel", 2);

    // Initialize other variables
    Odom_quat_get_.x = 0;
    Odom_quat_get_.y = 0;
    Odom_quat_get_.z = 0;
    Odom_quat_get_.w = 0;

    terrain_dwz_filter_.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    RCLCPP_INFO(this->get_logger(), "\nSimulation started.\n\n");

    // Timer to replace the while loop (rate-based processing)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5), std::bind(&RobotSimulator::updateLoop, this));
  }

private:
  // Callback and timer functions
  void scanHandler(const sensor_msgs::msg::PointCloud2::SharedPtr scanIn)
  {
    if (!systemInited)
    {
        RCLCPP_INFO(rclcpp::get_logger("scan_handler"), "Hold on, initializing!!");
        systemInitCount++;
        if (systemInitCount > systemDelay)
        {
            systemInited = true;
            RCLCPP_INFO(rclcpp::get_logger("scan_handler"), "Initialization done!!");
        }
        return;
    }

    double scanTime = scanIn->header.stamp.sec + scanIn->header.stamp.nanosec * 1e-9;

    if (odomSendIDPointer < 0)
    {
        return;
    }
    while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
           odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
    {
        odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
    }

    double odomRecTime = odomTime.seconds();
    float vehicleRecX = vehicleX;
    float vehicleRecY = vehicleY;
    float vehicleRecZ = vehicleZ;
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecYaw = vehicleYaw;
    float terrainRecRoll = terrainRoll;
    float terrainRecPitch = terrainPitch;

    if (use_gazebo_time)
    {
        odomRecTime = odomTimeStack[odomRecIDPointer];
        vehicleRecX = vehicleXStack[odomRecIDPointer];
        vehicleRecY = vehicleYStack[odomRecIDPointer];
        vehicleRecZ = vehicleZStack[odomRecIDPointer];
        vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
        vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
        vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
        terrainRecRoll = terrainRollStack[odomRecIDPointer];
        terrainRecPitch = terrainPitchStack[odomRecIDPointer];
    }

    float sinTerrainRecRoll = sin(terrainRecRoll);
    float cosTerrainRecRoll = cos(terrainRecRoll);
    float sinTerrainRecPitch = sin(terrainRecPitch);
    float cosTerrainRecPitch = cos(terrainRecPitch);

    pcl::fromROSMsg(*scanIn, *scanData);
    pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

    // Publish registered scan
    sensor_msgs::msg::PointCloud2 scanData2;
    pcl::toROSMsg(*scanData, scanData2);
    pcl_ros::transformPointCloud("map", *scanIn, scanData2, tf_buffer_);

    scanData2.header.stamp = rclcpp::Time(odomRecTime);
    scanData2.header.frame_id = "map";
    pubScanPointer->publish(scanData2);
  }

  void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr terrainCloud2)
  {

      if (!adjustZ && !adjustIncl)
      {
        return;
      }
    
      terrainCloud->clear();
      pcl::fromROSMsg(*terrainCloud2, *terrainCloud);
    
      pcl::PointXYZI point;
      terrainCloudIncl->clear();
      int terrainCloudSize = terrainCloud->points.size();
      double elevMean = 0;
      int elevCount = 0;
      bool terrainValid = true;
      for (int i = 0; i < terrainCloudSize; i++)
      {
        point = terrainCloud->points[i];
    
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
    
        if (dis < terrainRadiusZ)
        {
          if (point.intensity < groundHeightThre)
          {
            elevMean += point.z;
            elevCount++;
          }
          else
          {
            terrainValid = false;
          }
        }
    
        if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
        {
          terrainCloudIncl->push_back(point);
        }
      }
    
      if (elevCount >= minTerrainPointNumZ)
        elevMean /= elevCount;
      else
        terrainValid = false;
    
      if (terrainValid && adjustZ)
      {
        terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
      }
    
      terrainCloudDwz->clear();
      terrainDwzFilter.setInputCloud(terrainCloudIncl);
      terrainDwzFilter.filter(*terrainCloudDwz);
      int terrainCloudDwzSize = terrainCloudDwz->points.size();
    
      if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
      {
        return;
      }
    
      cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
      cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
      cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
      cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));
    
      int inlierNum = 0;
      matX.at<float>(0, 0) = terrainPitch;
      matX.at<float>(1, 0) = terrainRoll;
      for (int iterCount = 0; iterCount < 5; iterCount++)
      {
        int outlierCount = 0;
        for (int i = 0; i < terrainCloudDwzSize; i++)
        {
          point = terrainCloudDwz->points[i];
    
          matA.at<float>(i, 0) = -point.x + vehicleX;
          matA.at<float>(i, 1) = point.y - vehicleY;
          matB.at<float>(i, 0) = point.z - elevMean;
    
          if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
                   matB.at<float>(i, 0)) > InclFittingThre &&
              iterCount > 0)
          {
            matA.at<float>(i, 0) = 0;
            matA.at<float>(i, 1) = 0;
            matB.at<float>(i, 0) = 0;
            outlierCount++;
          }
        }
    
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
    
        if (inlierNum == terrainCloudDwzSize - outlierCount)
          break;
        inlierNum = terrainCloudDwzSize - outlierCount;
      }
    
      if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
          fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
      {
        terrainValid = false;
      }
    
      if (terrainValid && adjustIncl)
      {
        terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
        terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
      }

  }

  // Subscribe to the original odom topic then change the coordinate system of the odom
  void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
  {
    Odom_quat_get = odomIn->pose.pose.orientation;
    Odom_pose_get = odomIn->pose.pose.position;
  }

  void speedHandler(const geometry_msgs::msg::TwistStamped::ConstPtr& speedIn)
  {
      geometry_msgs::msg::Twist speed_temp = speedIn->twist;
    
      // speed_temp.linear.x = speedIn->twist.linear.x;
      // speed_temp.linear.y = speedIn->twist.linear.y;
      // speed_temp.linear.z = speedIn->twist.linear.z;
      // speed_temp.angular.x = speedIn->twist.angular.x;
      // speed_temp.angular.y =speedIn->twist.angular.y;
      // speed_temp.angular.z =speedIn->twist.angular.z;
    
      pubRobotSpeed->publish(speed_temp);
}  

  void updateLoop()
  {
    rclcpp::Time current_time = this->get_clock()->now();

    // Try to lookup the transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_.lookupTransform("map", "velodyne", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    //vehicleZ = terrainZ + vehicleHeight;

    rclcpp::Time odomTimeRec = odomTime;
    odomTime = this->get_clock()->now();
    if (odomTime == odomTimeRec) {
        odomTime += rclcpp::Duration::from_seconds(0.005);
    }
    
    // Convert quaternion and extract roll, pitch, yaw
    tf2::Quaternion quat;
    tf2::fromMsg(Odom_quat_get, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.seconds();
    vehicleXStack[odomSendIDPointer] = Odom_pose_get.x;
    vehicleYStack[odomSendIDPointer] = Odom_pose_get.y;
    vehicleZStack[odomSendIDPointer] = Odom_pose_get.z;
    vehicleRollStack[odomSendIDPointer] = roll;
    vehiclePitchStack[odomSendIDPointer] = pitch;
    vehicleYawStack[odomSendIDPointer] = yaw;
    terrainRollStack[odomSendIDPointer] = roll;
    terrainPitchStack[odomSendIDPointer] = pitch;

    // Assign value to odometry topic 
    nav_msgs::msg::Odometry odom_data;
    odom_data.header.stamp = odomTime;
    odom_data.pose.pose.orientation = Odom_quat_get;
    odom_data.pose.pose.position = Odom_pose_get;
    
    // Update z position based on transform
    Odom_pose_get.z = transform.transform.translation.z;
    
    bool quat_valid = (Odom_quat_get.x != 0 || 
                       Odom_quat_get.y != 0 || 
                       Odom_quat_get.z != 0 || 
                       Odom_quat_get.w != 0 ||
                       Odom_quat_get.w != 0) ? 1:0;

    if (quat_valid) {
        // Publish odometry
        pub_vehicle_odom_->publish(odom_data);
    
        // Create and publish transform from map to sensor at 200Hz
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = odomTime;  // Use ROS 2 time
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "sensor";
        
        // Set the translation (origin)
        odom_trans.transform.translation.x = Odom_pose_get.x;
        odom_trans.transform.translation.y = Odom_pose_get.y;
        odom_trans.transform.translation.z = Odom_pose_get.z;
    
        // Set the rotation (quaternion)
        odom_trans.transform.rotation.x = Odom_quat_get.x;
        odom_trans.transform.rotation.y = Odom_quat_get.y;
        odom_trans.transform.rotation.z = Odom_quat_get.z;
        odom_trans.transform.rotation.w = Odom_quat_get.w;
    
        // Broadcast the transform using tf2_ros::TransformBroadcaster
        tf_broadcaster_.sendTransform(odom_trans);
        }

    }  

  // Members
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_terrain_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_speed_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_vehicle_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_speed_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::Quaternion Odom_quat_get_;
  geometry_msgs::msg::Point Odom_pose_get_;

  pcl::VoxelGrid<pcl::PointXYZI> terrain_dwz_filter_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSimulator>());
  rclcpp::shutdown();
  return 0;
}