import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Convert OpenCV image to ROS2 Image message
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2
import numpy as np
import os
import xml.etree.ElementTree as ET


def get_network_interface_name():
    """Extract network interface name from CYCLONEDDS_URI environment variable"""
    cyclonedds_uri = os.environ.get('CYCLONEDDS_URI')
    if not cyclonedds_uri:
        return None
    
    try:
        # Parse the XML content
        root = ET.fromstring(cyclonedds_uri)
        
        # Find the NetworkInterface element and get its name attribute
        network_interface = root.find('.//NetworkInterface')
        if network_interface is not None:
            return network_interface.get('name')
        else:
            return None
    except ET.ParseError as e:
        print(f"Error parsing CYCLONEDDS_URI XML: {e}")
        return None

class FrontCameraNode(Node):
    def __init__(self):
        super().__init__("front_camera_node")
        self.publisher_ = self.create_publisher(Image, "/front_camera/image_raw", 10)
        self.bridge = CvBridge()
        
        # Get network interface name
        network_interface = get_network_interface_name()
        if network_interface:
            self.get_logger().info(f"Using network interface: {network_interface}")
        else:
            self.get_logger().warning("Could not extract network interface name from CYCLONEDDS_URI")
            network_interface = ""  # Provide default empty string
        
        # Initialize video client
        try:
            channel_factory = ChannelFactoryInitialize(0, network_interface)
            self.client = VideoClient()
            self.client.SetTimeout(3.0)
            self.client.Init()
            self.get_logger().info("Video client initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize video client: {e}")
            self.client = None
        
        # Create timer for periodic image capture (30 FPS = ~33ms interval)
        self.timer = self.create_timer(0.033, self.capture_callback)
        self.get_logger().info("Front camera node started with timer-based capture")

    def capture_callback(self):
        """Callback function to capture and publish image data"""
        if self.client is None:
            self.get_logger().warn("Video client not available, skipping capture")
            return
        
        try:
            code, data = self.client.GetImageSample()

            # Check if we got valid data
            if code == 0 and data is not None:
                # Convert data to numpy array
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                
                if image is not None:
                    # Convert to ROS2 Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    self.publisher_.publish(ros_image)
                else:
                    self.get_logger().warn("Failed to decode image data")
            elif code != 0:
                self.get_logger().debug(f"GetImageSample returned code: {code}")
                
        except Exception as e:
            self.get_logger().error(f"Error in capture callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    front_camera_node = FrontCameraNode()
    
    try:
        rclpy.spin(front_camera_node)
    except KeyboardInterrupt:
        front_camera_node.get_logger().info("Shutting down front camera node...")
    except Exception as e:
        front_camera_node.get_logger().error(f"Unexpected error: {e}")
    finally:
        try:
            front_camera_node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            # Ignore shutdown errors as they're common when interrupting
            pass

if __name__ == "__main__":
    main()