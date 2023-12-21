// ROS Includes
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cv_bridge/cv_bridge.h>

// Others
#include <opencv2/opencv.hpp>

class MappingScreenRenderer {

public:
  MappingScreenRenderer();

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg_map);

  void timerCallback(const ros::TimerEvent &event);

  // Others
  cv::Mat m_map_render;
  cv::Mat m_robot_icon;
  cv::Mat m_robot_icon_rotated;
  bool m_start_timer;

  // ROS Node Handles
  ros::NodeHandle m_nh, m_nh_private;

  // ROS Node Parameters
  std::string m_robot_icon_path;

  // ROS Subscribers
  ros::Subscriber m_subscriber_map;

  // ROS Publishers
  ros::Publisher m_publisher_rendered_image;

  // ROS Timer
  ros::Timer m_timer_publish;

  // ROS Others
  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener{m_tf_buffer};
  geometry_msgs::TransformStamped m_robot_to_map_tf;
  nav_msgs::MapMetaData m_map_meta_data;
};