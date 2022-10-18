#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h>  
#include <limits>
#include <vector>
#include <string>


using namespace std::chrono_literals;

//creates a DepthToImageProjection class that subclasses the generic rclcpp::Node base class.
class DepthToImageProjection : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		DepthToImageProjection() : Node("img_3d_to_2d_projection") {
			proj_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("img_3d_to_2d_proj", 10);
						
			lidar_to_mmwave_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/lidar_to_mmwave_pcl",	10,
			std::bind(&DepthToImageProjection::OnDepthMsg, this, std::placeholders::_1));

			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/cable_camera/image_raw",	10,
			std::bind(&DepthToImageProjection::OnCameraMsg, this, std::placeholders::_1));
		}

		~DepthToImageProjection() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down img_3d_to_2d_projection..");
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr proj_img_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_subscription_;

		std::vector<float> objects_dists;
		std::vector<float> objects_xz_angle;
		std::vector<float> objects_yz_angle;
		int closest_idx;

		void OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);
};



// mmwave message callback function
void DepthToImageProjection::OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg){
	// read PointCloud2 msg data
	int pcl_size = _msg->width;
	uint8_t *ptr = _msg->data.data();
	const uint32_t  POINT_STEP = 12;
	std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;
	for (size_t i = 0; i < pcl_size; i++)
	{
		pcl_x.push_back(*(reinterpret_cast<float*>(ptr + 0)));
		pcl_y.push_back(*(reinterpret_cast<float*>(ptr + 4)));
		pcl_z.push_back(*(reinterpret_cast<float*>(ptr + 8)));
		ptr += POINT_STEP;
	}
	float closest_dist = std::numeric_limits<float>::max(); 
	float current_dist = 0;
	int closest_dist_idx = 0;
	// find closest point in pointcloud msg, store all angles and dists
	this->objects_dists.clear();
	this->objects_xz_angle.clear();
	this->objects_yz_angle.clear();
	this->closest_idx = 0;
	if(pcl_size > 0){
		for (int i = 0; i < pcl_size; i++)
		{
			current_dist = sqrt( pow(pcl_x.at(i), 2) + pow(pcl_y.at(i), 2) + pow(pcl_z.at(i), 2) );

			this->objects_dists.push_back(current_dist);
			this->objects_xz_angle.push_back( asin(pcl_x.at(i) / sqrt(pow(pcl_x.at(i),2) + pow(pcl_z.at(i),2))) );
			this->objects_yz_angle.push_back( asin(pcl_y.at(i) / sqrt(pow(pcl_y.at(i),2) + pow(pcl_z.at(i),2))) );

			if( current_dist < closest_dist ){
				closest_dist = current_dist;
				this->closest_idx = i;
			}
		}
		/*std::cout << "closest dist: " << this->objects_dists.at(this->closest_idx) << std::endl;
		std::cout << "closest xz: " << this->objects_xz_angle.at(this->closest_idx) << std::endl;
		std::cout << "closest yz: " << this->objects_yz_angle.at(this->closest_idx) << std::endl;*/
	}
}


// calculate pixel coordinates for 3d->2d projection
void DepthToImageProjection::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){
	float img_hfov = 1.3962634016;
	float h_focal_length = (_msg->width * 0.5) / tan(img_hfov * 0.5 ); // in pixels
	// find horisontal and vertical pixel where nearest object would be
	std::vector<float> x_px_vec;
	std::vector<float> y_px_vec;

	for (int i = 0; i < this->objects_dists.size(); i++)
	{
		// horizontal pixel
		float x_depth = sin(this->objects_xz_angle.at(i)) * this->objects_dists.at(i);
		float y_depth = cos(this->objects_xz_angle.at(i)) * this->objects_dists.at(i);
		float xy_ratio = std::numeric_limits<float>::max(); 
		if( y_depth != 0 ){
			xy_ratio = x_depth/y_depth;
		}
		x_px_vec.push_back( -1 * xy_ratio * h_focal_length  + _msg->width/2); // -1 to mirror (pinhole stuff)
		
		// vertical pixel
		x_depth = sin(this->objects_yz_angle.at(i)) * this->objects_dists.at(i); 
		y_depth = cos(this->objects_yz_angle.at(i)) * this->objects_dists.at(i);
		xy_ratio = std::numeric_limits<float>::max(); 
		if( y_depth != 0 ){
			xy_ratio = x_depth/y_depth;
		}
		y_px_vec.push_back( -1 * xy_ratio * h_focal_length + _msg->height/2); // -1 to mirror (pinhole stuff)
	}
	if (this->objects_dists.size() > 0)
	{
		RCLCPP_INFO(this->get_logger(),  "\n Shortest dist pixel: \n x_px: %f, \n y_px: %f", x_px_vec.at(this->closest_idx), y_px_vec.at(this->closest_idx));
	}
	else {
		RCLCPP_INFO(this->get_logger(),  "\n No points in pointcloud");

	}
	

	int square_radius = 15; // "radius" of drawn square
	int square_diameter = square_radius*2;
	int num_channels = 3; // rgb
	// draw squares
	for (int i = 0; i < this->objects_dists.size(); i++){
		for (int y = 0; y < square_diameter; y++){
			for (int x = 0; x < square_diameter; x++){
				for (int channel = 0; channel < num_channels; channel++){
					if (x_px_vec.at(i) > 0 && x_px_vec.at(i) < _msg->width && y_px_vec.at(i) > 0 && y_px_vec.at(i) < _msg->height)
					{
						if (i == this->closest_idx)
						{		
							_msg->data.at(  (_msg->width * ((int(y_px_vec.at(i))-((square_radius)-1))+y) + 
								((int(x_px_vec.at(i))-((square_radius)-1))+x)) * num_channels + channel ) = 0;
						} 
						else 
						{
							_msg->data.at(  (_msg->width * ((int(y_px_vec.at(i))-((square_radius)-1))+y) + 
								((int(x_px_vec.at(i))-((square_radius)-1))+x)) * num_channels + channel ) = 255;
						}
					}	
				}
			}
		}
	}

	x_px_vec.clear();
	y_px_vec.clear();

	// create and publish new image msg
	auto img_pub_msg = sensor_msgs::msg::Image();
	img_pub_msg.header = std_msgs::msg::Header();
	img_pub_msg.header.stamp = this->now();
	std::string frameID = "map";
	img_pub_msg.header.frame_id = frameID;
	img_pub_msg.height = _msg->height;
	img_pub_msg.width = _msg->width;
	img_pub_msg.encoding = _msg->encoding;
	img_pub_msg.is_bigendian = _msg->is_bigendian;
	img_pub_msg.step = _msg->step;
	img_pub_msg.data = _msg->data;
	this->proj_img_publisher_->publish(img_pub_msg);
}

	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting DepthToImageProjection node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DepthToImageProjection>());

	rclcpp::shutdown();
	return 0;
}
