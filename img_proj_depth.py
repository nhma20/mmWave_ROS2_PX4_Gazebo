#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class ImageProjectDepthNode(Node):
    def __init__(self):
        super().__init__("img_proj_depth")
        self.altered_img_publisher_ = self.create_publisher(
            Image, "/cable_camera/img_proj_depth", 10)
            
        self.horisontal_pixel_subscription_ = self.create_subscription(
            Float32, '/horisontal_cable_pixel', self.listener_callback, 10)
        self.horisontal_pixel_subscription_           
            
    def publish_altered_image(self):
        msg = Float32()
        msg.data = temperature
        self.altered_img_publisher_.publish(msg)
        
    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%f"' % msg.data) # CHANGE
             
            img_msg = Image()
            self.altered_img_publisher_.publish(img_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageProjectDepthNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
