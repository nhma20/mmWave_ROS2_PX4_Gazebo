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
import matplotlib.pyplot as plt


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
		self.horisontal_pixel = msg.data # 1D list of 6220800 elements (1920 x 1080 x 3)	
		
	def img_msg_callback(self, msg):
		corrected_y_loc = (0 + msg.height/2)
		corrected_x_loc = (self.horisontal_pixel + (msg.width/2))

		self.get_logger().info('\n Horisontal pixel: "%f" \n Vertical pixel: "%f"' % (corrected_x_loc, corrected_y_loc))

		## with deserialization of data 
		"""np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) #deserialize image msg
		print("np_img shape: ", np_img.shape)
		rect_start_point = (round(corrected_x_loc-10), round((msg.width/2)-10))
		rect_end_point = (round(corrected_x_loc+10), round((msg.width/2)+10))
		color = (255,255,255)
		thickness = -1 # fill
		draw_img = cv.rectangle(np_img, rect_start_point, rect_end_point, color, thickness)
		flat_img = draw_img.flatten().tolist() # very slow"""


		## without deserializing data
		square_radius = 15
		num_channels = 3
		for y in range(square_radius*2):
			for x in range(square_radius*2):
				for channel in range(3):
					if(corrected_x_loc > 0 and corrected_x_loc < msg.width and corrected_y_loc > 0 and corrected_y_loc < msg.height):
						msg.data[ round( msg.width * ((corrected_y_loc-((square_radius)-1))+y) * num_channels 
									+ ((corrected_x_loc-((square_radius)-1))+x) * num_channels + channel ) ] = 255


		img_pub_msg = Image()
		img_pub_msg.header = std_msgs.msg.Header()
		img_pub_msg.header.stamp = self.get_clock().now().to_msg()
		img_pub_msg.header.frame_id = 'map'
		img_pub_msg.height = msg.height
		img_pub_msg.width = msg.width
		img_pub_msg.encoding = msg.encoding
		img_pub_msg.is_bigendian = msg.is_bigendian
		img_pub_msg.step = msg.step
		img_pub_msg.data = msg.data#flat_img#img_copy
		self.altered_img_publisher_.publish(img_pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProjectDepthNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
