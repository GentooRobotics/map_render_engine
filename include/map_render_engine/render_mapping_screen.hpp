// ROS Includes
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

// Others
#include <opencv2/opencv.hpp>

class MappingScreenRenderer 
{ 

public:

    ////////////////////////////////////////////////////////////////
    // Constructor
    MappingScreenRenderer();

    ////////////////////////////////////////////////////////////////

private:
    ////////////////////////////////////////////////////////////////
    // Callback Functions
    void synchronizedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg_map, const sensor_msgs::LaserScan::ConstPtr& msg_laser);

    void timerCallback(const ros::TimerEvent& event);

    ////////////////////////////////////////////////////////////////
    // ROS 

    // ROS Node Handles
    ros::NodeHandle m_nh, m_nh_private;

    // ROS Subscribers
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, sensor_msgs::LaserScan> MappingScreenSyncPolicy;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> m_subscriber_map;
    message_filters::Subscriber<sensor_msgs::LaserScan> m_subscriber_laser;
    message_filters::Synchronizer<MappingScreenSyncPolicy> m_synchronizer;

    // ROS Publishers 
    ros::Publisher m_publisher_rendered_image;

    // ROS Timer
    ros::Timer m_timer_publish;

    ////////////////////////////////////////////////////////////////
    // Others
    cv::Mat m_map_render;
    ////////////////////////////////////////////////////////////////

};