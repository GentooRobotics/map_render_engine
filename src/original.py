import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion

import cv2
import numpy as np

import utils


class SyncTopics(Node):
    def __init__(self):
        super().__init__("map_renderer")
        self.map_sub = Subscriber(self, OccupancyGrid, "/map")
        self.laser_scan_sub = Subscriber(self, LaserScan, "/scan")
        self.map_with_laser_pub = self.create_publisher(Image, "/map_with_laser", 1)
        self.map_viz_local = self.create_publisher(Image, "/map_viz_local", 1)
        self.cv_bridge = CvBridge()
        self.robot_image = cv2.imread('./robot.png', cv2.IMREAD_UNCHANGED)

        # TF2 ROS
        self.tf_buffer = tf2_ros.buffer.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.synced_subs = ApproximateTimeSynchronizer(
            [self.map_sub, self.laser_scan_sub], 10, slop=0.1
        )
        self.synced_subs.registerCallback(self.callback)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.map_msg = None
        self.map_image = np.array([])
        self.robot_size = 0.3
        self.robot_min_pixel_size = 30
        self.local_map_shape = (640,360)
    
    def get_robot_pose_in_map(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            transform = transform_stamped.transform
            (roll, pitch, yaw) = euler_from_quaternion([transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w])

            x = transform.translation.x
            y = transform.translation.y

            return x,y,yaw

        except Exception as e:
            print(f"Error: {e}")

    
    def timer_callback(self):
        if self.map_msg == None or not self.map_image.size:
            return

        map_resolution = self.map_msg.info.resolution 
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        map_origin_x = self.map_msg.info.origin.position.x
        map_origin_y = self.map_msg.info.origin.position.y

        # get robot pose
        pose = self.get_robot_pose_in_map()
        if pose == None:
            return 
        x,y,yaw = pose 
        robot_image = utils.rotate_image(self.robot_image, yaw)

        # calc scale factor
        robot_pixel_size = round(self.robot_size/map_resolution)
        scale_factor = 1
        if (robot_pixel_size < self.robot_min_pixel_size):
            scale_factor = self.robot_min_pixel_size /robot_pixel_size
            robot_pixel_size = self.robot_min_pixel_size
            
        robot_image = cv2.resize(robot_image, (robot_pixel_size, robot_pixel_size))
        final_image = cv2.resize(self.map_image.copy(), (int(width * scale_factor), int(height * scale_factor)))

        x_image = (x - map_origin_x) / map_resolution
        y_image = (y - map_origin_y) / map_resolution
        x_image = round(x_image*scale_factor - robot_image.shape[1]/2)
        y_image = round((self.map_msg.info.height-y_image) * scale_factor - robot_image.shape[0]/2)
        
        # merge global map image with robot image
        map_with_robot_image = utils.alphaMerge(robot_image, final_image, y_image, x_image)
        global_image_msg = self.cv_bridge.cv2_to_imgmsg(map_with_robot_image, encoding="bgr8")
        self.map_with_laser_pub.publish(global_image_msg)
        # cv2.imwrite("./global.png", map_with_robot_image)

        # local map image
        max_shape = max(self.local_map_shape[0], self.local_map_shape[1])
        cropped = utils.pad_and_crop(map_with_robot_image, (x_image,y_image), max_shape)
        rotated = utils.rotate_image(cropped, np.pi/2 - yaw)
        local_map_image = utils.crop_to_size(rotated, self.local_map_shape)
        local_image_msg = self.cv_bridge.cv2_to_imgmsg(local_map_image, encoding="bgr8")
        self.map_viz_local.publish(local_image_msg)
        # cv2.imwrite("./local.png", local_map_image)

    def callback(self, map_msg, laser_msg):
        self.map_msg = map_msg  
        self.map_image = self.plot_laser_on_map_image(map_msg, laser_msg)

    def create_map_image(self, map_msg):
        # Create an image from the occupancy grid map
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # Convert -1 (unknown) to gray, 0 (free) to white, and 100 (occupied) to black
        map_image = np.zeros((map_msg.info.height, map_msg.info.width, 3), dtype=np.uint8)
        gray_mask = map_data == -1
        white_mask = map_data == 0
        black_mask = map_data == 100

        # Set colors based on masks
        map_image[gray_mask] = (128, 128, 128)  # Gray for unknown
        map_image[white_mask] = (255, 255, 255)  # White for free
        map_image[black_mask] = (0, 0, 0)  # Black for occupied

        # map_image = np.flip(map_image, 1)
        map_image = np.flip(map_image, 0)
        return map_image
    
    def plot_laser_on_map_image(self, map_msg, laser_msg):
        map_image = self.create_map_image(map_msg)    

        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                "rplidar_link", "map", laser_msg.header.stamp
            )

            for i, range_value in enumerate(laser_msg.ranges):
                if range_value < laser_msg.range_max:
                    angle = laser_msg.angle_min + i * laser_msg.angle_increment
                    laser_point = PointStamped()
                    laser_point.header.frame_id = laser_msg.header.frame_id
                    laser_point.point.x = range_value * np.cos(angle)
                    laser_point.point.y = range_value * np.sin(angle)
                    laser_point.point.z = 0.0

                    # Transform the point to map frame
                    transformed_point = self.tf_buffer.transform(
                        laser_point, "map", Duration(seconds=0)
                    )

                    pixel_x = round(
                        (transformed_point.point.x - map_msg.info.origin.position.x) / map_msg.info.resolution
                    ) 
                    pixel_y = self.map_msg.info.height - round(
                        (transformed_point.point.y - map_msg.info.origin.position.y) / map_msg.info.resolution
                    ) 
                
                    if 0 <= pixel_x < map_msg.info.width and 0 <= pixel_y < map_msg.info.height:
                        map_image[pixel_y, pixel_x, 0] = 255
                        map_image[pixel_y, pixel_x, 1] = 0
                        map_image[pixel_y, pixel_x, 2] = 0

        except Exception as e:
            print(f"Error: {e}")     

        return map_image   


def main(args=None):
    rclpy.init(args=args)
    node = SyncTopics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()