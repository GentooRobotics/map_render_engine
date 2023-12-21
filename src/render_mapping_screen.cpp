#include "map_render_engine/render_mapping_screen.hpp"

void rotateImage(cv::Mat &input, cv::Mat &output, double angle) {
  cv::Point2f center(input.cols / 2.0, input.rows / 2.0);
  cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angle, 1.0);

  cv::warpAffine(input, output, rotationMatrix, input.size());
}

MappingScreenRenderer::MappingScreenRenderer() : m_nh_private{"~"} {
  // ROS Node Parameters
  m_robot_icon_path =
      m_nh_private.param<std::string>("robot_icon", "robot.png");

  // Variables Initialization
  m_map_render = cv::Mat(300, 300, CV_8UC4);
  m_robot_icon = cv::imread(m_robot_icon_path, cv::ImreadModes::IMREAD_UNCHANGED);
  m_robot_icon_rotated = cv::imread(m_robot_icon_path, cv::ImreadModes::IMREAD_UNCHANGED);
  m_start_timer = false;

#ifdef DEBUG_MODE
  cv::imshow("Robot Icon", m_robot_icon);
  cv::waitKey(2);
#endif

  // ROS Subscribers
  m_subscriber_map = m_nh.subscribe<nav_msgs::OccupancyGrid>(
      "/map", 1, &MappingScreenRenderer::mapCallback, this);

  // ROS Publishers
  m_publisher_rendered_image =
      m_nh.advertise<sensor_msgs::Image>("mapping_screen", 1);

  ros::spin();
}

void MappingScreenRenderer::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &msg_map) {
  ////////////////////////////////////////////////////////////////
  // If Map Image Size Is Different, Resize Render

#ifdef DEBUG_MODE
  ROS_INFO("Map Callback Size: (%d, %d)", msg_map->info.width,
           msg_map->info.height);
#endif

  if (m_map_render.cols != msg_map->info.width ||
      m_map_render.rows != msg_map->info.height) {
    m_map_render = cv::Mat::zeros(
        cv::Size(msg_map->info.width, msg_map->info.height), CV_8UC4);
  }

#ifdef DEBUG_MODE
  ROS_INFO("Map Render Size: (%d, %d)", m_map_render.cols, m_map_render.rows);
#endif

  ////////////////////////////////////////////////////////////////
  // Generate Latest Map Render

  std::size_t rows = static_cast<std::size_t>(m_map_render.rows);
  std::size_t cols = static_cast<std::size_t>(m_map_render.cols);
  std::uint8_t *p; // unsigned char pointer to access value
  for (std::size_t i = 0; i < rows; ++i) {
    // flip image at the same time in axis 0
    // to account for Occupancy Grid & Image Axis Difference
    p = m_map_render.ptr<std::uint8_t>(rows - i - 1);
    for (size_t j = 0; j < cols; ++j) {
      std::int8_t current_data = msg_map->data[i * cols + j];

      // Unknown Pixels, Set to Gray
      if (current_data == -1) {
        p[j * 4 + 0] = static_cast<std::uint8_t>(128); // r
        p[j * 4 + 1] = static_cast<std::uint8_t>(128); // g
        p[j * 4 + 2] = static_cast<std::uint8_t>(128); // b
        p[j * 4 + 3] = static_cast<std::uint8_t>(255);   // a
      }
      // Known Pixels, Set to 0-255 based on intensity
      else {
        // Probability to Pixel Value Conversion
        static constexpr float scale_factor = 255. / 100.;
        p[j * 4 + 0] =
            255 - static_cast<std::uint8_t>(current_data * scale_factor); // r
        p[j * 4 + 1] =
            255 - static_cast<std::uint8_t>(current_data * scale_factor); // g
        p[j * 4 + 2] =
            255 - static_cast<std::uint8_t>(current_data * scale_factor); // b
        p[j * 4 + 3] = 255;                                               // a
      }
    }
  }

#ifdef DEBUG_MODE
  cv::imshow("Raw Map Render", m_map_render);
  cv::waitKey(1);
#endif

  ////////////////////////////////////////////////////////////////
  // Update Map Info
  m_map_meta_data = msg_map->info;

  if (m_robot_icon.cols != static_cast<int>(0.5 / m_map_meta_data.resolution) ||
      m_robot_icon.rows != static_cast<int>(0.5 / m_map_meta_data.resolution)) {
    cv::resize(m_robot_icon, m_robot_icon,
               cv::Size(0.5 / m_map_meta_data.resolution,
                        0.5 / m_map_meta_data.resolution));
    cv::resize(m_robot_icon_rotated, m_robot_icon_rotated,
               cv::Size(0.5 / m_map_meta_data.resolution,
                        0.5 / m_map_meta_data.resolution));
  }

#ifdef DEBUG_MODE
  cv::imshow("Robot Icon", m_robot_icon);
  cv::waitKey(2);
#endif

  ////////////////////////////////////////////////////////////////
  // Start Timer 
  if (!m_start_timer)
  {  
    m_timer_publish = m_nh.createTimer(
        ros::Duration(0.1), &MappingScreenRenderer::timerCallback, this);
    m_start_timer = true;
  }

  ////////////////////////////////////////////////////////////////
}

void MappingScreenRenderer::timerCallback(const ros::TimerEvent &event) {

  ////////////////////////////////////////////////////////////////
  // Get Robot to Map Transformation
  try {
    m_robot_to_map_tf = m_tf_buffer.lookupTransform(
        "map", "base_link", ros::Time::now(), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // get x, y, yaw
  double x = m_robot_to_map_tf.transform.translation.x;
  double y = m_robot_to_map_tf.transform.translation.y;
  tf2::Quaternion rotation_quaternion(m_robot_to_map_tf.transform.rotation.x,
                                      m_robot_to_map_tf.transform.rotation.y,
                                      m_robot_to_map_tf.transform.rotation.z,
                                      m_robot_to_map_tf.transform.rotation.w);
  tf2::Matrix3x3 rotation_matrix(rotation_quaternion);
  double roll, pitch, yaw;
  rotation_matrix.getRPY(roll, pitch, yaw);

#ifdef DEBUG_MODE
  ROS_INFO("Robot To Map TF (x, y, yaw): (%lf, %lf, %lf)", x, y, yaw);
#endif

  ////////////////////////////////////////////////////////////////
  // Rotate Robot Icon by Yaw
  static constexpr double rads_to_degrees = 180. / 3.14159;
  rotateImage(m_robot_icon, m_robot_icon_rotated, yaw * rads_to_degrees);

#ifdef DEBUG_MODE
  cv::imshow("Robot Icon Rotated", m_robot_icon_rotated);
  cv::waitKey(2);
#endif

  ////////////////////////////////////////////////////////////////
  // Add Robot Pose on Map Render
  // Calculate Robot Top Left Corner in Image
  double x_image;
  double y_image;
  x_image = (x - m_map_meta_data.origin.position.x) / m_map_meta_data.resolution;
  y_image = (y - m_map_meta_data.origin.position.y) / m_map_meta_data.resolution;
  x_image = std::round(x_image - m_robot_icon_rotated.cols/2);
  y_image = std::round((m_map_meta_data.height - y_image)  - m_robot_icon_rotated.rows/2);

  // Copy Rotated Robot Icon To Map
  cv::Rect roi(x_image, y_image, m_robot_icon_rotated.cols, m_robot_icon_rotated.rows);
  cv::Mat original_region = m_map_render(roi).clone(); // part of the render that is going to replaced by robot icon
  cv::Mat blended_region;

  std::size_t rows = static_cast<std::size_t>(m_map_render(roi).rows);
  std::size_t cols = static_cast<std::size_t>(m_map_render(roi).cols);
  std::uint8_t *p; // unsigned char pointer to access value
  std::uint8_t *p2; // unsigned char pointer to access value
  for (std::size_t i = 0; i < rows; ++i) {
    p = m_map_render(roi).ptr<std::uint8_t>(i);
    p2 = m_robot_icon_rotated.ptr<std::uint8_t>(i);
    for (size_t j = 0; j < cols; ++j) {
      // if robot icon's is not transparent
      // copy pixel value to map render
      if (p2[j * 4 + 3] != 0) 
      {
        p[j * 4 + 0] = p2[j * 4 + 0];
        p[j * 4 + 1] = p2[j * 4 + 1];
        p[j * 4 + 2] = p2[j * 4 + 2];
      }
    }
  } 

#ifdef DEBUG_MODE
  cv::imshow("Final Map Render", m_map_render);
  cv::waitKey(1);
#endif

  ////////////////////////////////////////////////////////////////
  // Clean Up
  original_region.copyTo(m_map_render(roi)); // replace the roi with original region 

  ////////////////////////////////////////////////////////////////
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_screen_renderer");

  MappingScreenRenderer mapping_screen_renderer;

  return 0;
}