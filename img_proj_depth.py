#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import std_msgs.msg
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
import copy


class ImageProjectDepthNode(Node):
	def __init__(self):
		super().__init__("img_proj_depth")

		self.horisontal_pixel = 0

		self.altered_img_publisher_ = self.create_publisher(
		Image, "/cable_camera/img_proj_depth", 10)

		self.horisontal_pixel_subscription_ = self.create_subscription(
		Float32, '/horisontal_cable_pixel', self.pixel_val_callback, 10)
		self.horisontal_pixel_subscription_   

		self.cable_cam_img_subscription_ = self.create_subscription(
		Image, '/cable_camera/image_raw', self.img_msg_callback, 10)
		self.cable_cam_img_subscription_           

	def pixel_val_callback(self, msg):
		#self.get_logger().info('Horisontal pixel: "%f"' % msg.data)
		self.horisontal_pixel = msg.data
		
	def img_msg_callback(self, msg):
		self.get_logger().info('Horisontal pixel: "%f"' % self.horisontal_pixel)
		self.get_logger().info('img width: "%f"' % msg.width) 
		#blank_image = np.zeros((msg.height,msg.width,3), np.uint8)
		img = msg.data # 6220800 elements (width x height x channnels)	
		img_copy = copy.deepcopy(msg.data)
		print(msg.data[0], msg.data[1], msg.data[2])
		corrected_y_loc = self.horisontal_pixel + (msg.width/2)
		for y in range(msg.height):
			for x in range(msg.width):
				for channels in range(3):
					if (abs(x-corrected_y_loc)<20) and (abs(y-(msg.height/2))<20):
						img_copy[y*msg.width*3+x*3+channels] = 255
						#print("changing..")
					else:
					#	img_copy[y*msg.width*3+x*3+channels] = 0
					#	print(y*msg.width*3+x*3+channels)
						pass # keep orignal color
					
		img_pub_msg = Image()
		img_pub_msg.header = std_msgs.msg.Header()
		img_pub_msg.header.stamp = self.get_clock().now().to_msg()
		img_pub_msg.header.frame_id = 'map'
		img_pub_msg.height = msg.height
		img_pub_msg.width = msg.width
		img_pub_msg.encoding = msg.encoding
		img_pub_msg.is_bigendian = msg.is_bigendian
		img_pub_msg.step = msg.step
		img_pub_msg.data = img_copy
		self.altered_img_publisher_.publish(img_pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProjectDepthNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
