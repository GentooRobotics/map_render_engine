#include "map_render_engine/render_mapping_screen.hpp"

MappingScreenRenderer::MappingScreenRenderer() :
m_nh_private{"~"},
m_synchronizer(MappingScreenSyncPolicy(10), m_subscriber_map, m_subscriber_laser)
{
    // Variables Initialization
    m_map_render = cv::Mat(300,300, CV_8UC3);

    // ROS Subscribers
    m_subscriber_map.subscribe(m_nh, "/map", 10);
    m_subscriber_laser.subscribe(m_nh, "/scan", 10);
    m_synchronizer.registerCallback(boost::bind(&MappingScreenRenderer::synchronizedCallback, this, _1, _2));

    // ROS Publishers
    m_publisher_rendered_image = m_nh.advertise<sensor_msgs::Image>("mapping_screen", 1);

    // ROS Timer 
    m_timer_publish = m_nh.createTimer(ros::Duration(0.1), &MappingScreenRenderer::timerCallback, this);


    ros::spin();
}

void MappingScreenRenderer::synchronizedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg_map, const sensor_msgs::LaserScan::ConstPtr& msg_laser)
{
    ////////////////////////////////////////////////////////////////
    // If Map Image Size Is Different, Resize Render

    #ifdef DEBUG_MODE
    ROS_INFO("Map Callback Size: (%d, %d)", msg_map->info.width, msg_map->info.height);
    #endif

    if (m_map_render.cols != msg_map->info.width || m_map_render.rows != msg_map->info.height)
    {
        m_map_render = cv::Mat::zeros(cv::Size(msg_map->info.width, msg_map->info.height), CV_8UC3);
    }

    #ifdef DEBUG_MODE
    ROS_INFO("Map Render Size: (%d, %d)", m_map_render.cols, m_map_render.rows);
    #endif 

    ////////////////////////////////////////////////////////////////
    // Generate Latest Map Render

    std::size_t rows = static_cast<std::size_t>(m_map_render.rows);
    std::size_t cols = static_cast<std::size_t>(m_map_render.cols);
    std::uint8_t* p; // unsigned char pointer to access value
    for(std::size_t i=0; i<rows; ++i)
    {
        // flip image at the same time in axis 0
        p = m_map_render.ptr<std::uint8_t>(rows - i - 1);
        for (size_t j=0; j<cols; ++j)
        {
            std::int8_t current_data = msg_map->data[i*cols + j];
            
            // Unknown Pixels, Set to Gray
            if (current_data == -1)
            {
                p[j*3 + 0] = static_cast<std::uint8_t>(128);
                p[j*3 + 1] = static_cast<std::uint8_t>(128);
                p[j*3 + 2] = static_cast<std::uint8_t>(128);
            }
            // Known Pixels, Set to 0-255 based on intensity
            else
            {
                // Probability to Pixel Value Conversion
                constexpr float scale_factor = 255./100.;
                p[j*3 + 0] = 255 - static_cast<std::uint8_t>(current_data*scale_factor);
                p[j*3 + 1] = 255 - static_cast<std::uint8_t>(current_data*scale_factor);
                p[j*3 + 2] = 255 - static_cast<std::uint8_t>(current_data*scale_factor);
            }
        }
    }

    #ifdef DEBUG_MODE
    cv::imshow("Raw Map Render", m_map_render);
    cv::waitKey(1);
    #endif 
    
    ////////////////////////////////////////////////////////////////



}

void MappingScreenRenderer::timerCallback(const ros::TimerEvent& event)
{
    // std::cout << "Timer Callback\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping_screen_renderer");

    MappingScreenRenderer mapping_screen_renderer;

    return 0;
}