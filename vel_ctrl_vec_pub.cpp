#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
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

//creates a VelocityControlVectorAdvertiser class that subclasses the generic rclcpp::Node base class.
class VelocityControlVectorAdvertiser : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		VelocityControlVectorAdvertiser() : Node("vel_ctrl_vect_advertiser") {
			publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("vel_ctrl_vect_topic", 10);
			
			hor_pix_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/horisontal_cable_pixel", 10);
			
			lidar_to_mmwave_pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_to_mmwave_pcl", 10);

			lidar_to_mmwave_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/lidar_to_mmwave_pcl",	10,
			std::bind(&VelocityControlVectorAdvertiser::OnDepthMsg, this, std::placeholders::_1));

			subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/dist_sensor/laser_scan",	10,
			std::bind(&VelocityControlVectorAdvertiser::lidar_to_mmwave_pcl, this, std::placeholders::_1));
			
			camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/cable_camera/image_raw",	10,
			std::bind(&VelocityControlVectorAdvertiser::OnCameraMsg, this, std::placeholders::_1));
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
		rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hor_pix_publisher_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_subscription_;

		float shortestDistAngle_;
		float shortestDist_;
		std::vector<float> objects_dist;
		std::vector<float> objects_angl;
		int callback_count = 0;
		void OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
		void OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg);
		void lidar_to_mmwave_pcl(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
		void VelocityDroneControl(float xv, float yv, float zv);
		bool beginHover();
};


// Control drone velocities
void VelocityControlVectorAdvertiser::VelocityDroneControl(float xv, float yv, float zv){
	auto vel_ctrl_vect = px4_msgs::msg::DebugVect();
	vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	std::string name = "test";
	std::copy(name.begin(), name.end(), vel_ctrl_vect.name.begin());
	vel_ctrl_vect.x = xv;
	vel_ctrl_vect.y = yv;
	vel_ctrl_vect.z = zv;
	this->publisher_->publish(vel_ctrl_vect);
}


// mmwave message callback function
void VelocityControlVectorAdvertiser::OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg){
	float shortestDistIdxAngle = this->shortestDistAngle_ ;

	auto vel_ctrl_vect = px4_msgs::msg::DebugVect();
	vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	std::string name = "test";
	std::copy(name.begin(), name.end(), vel_ctrl_vect.name.begin());
	
	bool hovering = true; //beginHover();

	float control_distance = 0.5; // desired distance to cable (meters)
	float control_angle = 0.0; // desired angle to cable (rad)
	float p_dist = 0.5; // proportional gain for distance controller
	float p_angle = 3.0; // proportional gain for angle controller

	float shortest_dist = this->shortestDist_;
	if(hovering == true){
		if(shortest_dist < 10){
			// create velocity control vector to steer drone towards cable
			VelocityDroneControl(0, p_angle*(shortestDistIdxAngle-control_angle), -p_dist*(shortest_dist-control_distance));
		
		} else {	
			// control drone in square motion if nothing within lidar fov
			static int callbackscale = 5;
			std::cout << "CallbackCount" << callback_count << std::endl;
			if(callback_count < 100*callbackscale){
				VelocityDroneControl(NAN,NAN,-3.0); // go up
				std::cout << "START" << std::endl;
			} else if(callback_count % (100*callbackscale) < 25*callbackscale){
				VelocityDroneControl(0,3,0); // go east
				std::cout << "EAST" << std::endl;
			} else if(callback_count % (100*callbackscale) < 50*callbackscale){
				VelocityDroneControl(0,0,-3); // go up
				std::cout << "UP" << std::endl;
			} else if(callback_count % (100*callbackscale) < 75*callbackscale){
				VelocityDroneControl(0,-3,0); // go west
				std::cout << "WEST" << std::endl;
			} else if(callback_count % (100*callbackscale) > 74*callbackscale){
				VelocityDroneControl(0,0,3); // go down
				std::cout << "DOWN" << std::endl;
			} else{
				VelocityDroneControl(0,0,0); // do nothing
			}	
			callback_count++;
		}
	}
}

// Gets drone in hovering position. Returns true when done
bool VelocityControlVectorAdvertiser::beginHover(){
	static auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
	std::cout << "Elapsed time: " << elapsed_seconds.count() << std::endl;
	int wait_time_s = 6;
	if(double(elapsed_seconds.count()) > wait_time_s+3){
		return 1;
	}
	else if(double(elapsed_seconds.count()) > wait_time_s){
		VelocityDroneControl(0,0,0); // might not be a good idea
	}
	else if(double(elapsed_seconds.count()) > 2){
		VelocityDroneControl( 0, 0, (elapsed_seconds.count()-wait_time_s) );
	}
	else{
		VelocityDroneControl(0,0,0);
	}
	return 0;
}


// calculate pixel coordinates for 3d->2d projection
void VelocityControlVectorAdvertiser::OnCameraMsg(const sensor_msgs::msg::Image::SharedPtr _msg){
	float img_hfov = 1.3962634016;
	float h_focal_length = (_msg->width * 0.5) / tan(img_hfov * 0.5 ); // in pixels
	// publish horisontal pixel where nearest object would be
	float x_depth = sin(this->shortestDistAngle_) * this->shortestDist_;
	float y_depth = cos(this->shortestDistAngle_) * this->shortestDist_;
	float xy_ratio = std::numeric_limits<float>::max(); 
	if( y_depth != 0 ){
		xy_ratio = x_depth/y_depth;
	}
	float x_px = -1 * xy_ratio * h_focal_length; // -1 to mirror (pinhole stuff)
	
	// publish vertical pixel where nearest object would be
	 x_depth = sin(0) * this->shortestDist_; // 0 angle because lidar only 2d, no vertical fov
	 y_depth = cos(0) * this->shortestDist_;
	 xy_ratio = std::numeric_limits<float>::max(); 
	if( y_depth != 0 ){
		xy_ratio = x_depth/y_depth;
	}
	float y_px = -1 * xy_ratio * h_focal_length; // -1 to mirror (pinhole stuff)

	auto float32_msg = std_msgs::msg::Float32();
	float32_msg.data = x_px;
	this->hor_pix_publisher_->publish(float32_msg);
}


// converts lidar data to pointcloud of detected objects to simulate sparse mmwave
void VelocityControlVectorAdvertiser::lidar_to_mmwave_pcl(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
	float angle_increment = _msg->angle_increment;
	float angle_min = _msg->angle_min;
	float angle_max = _msg->angle_max;
	float range_max = _msg->range_max;
	float range_min = _msg->range_min;
	float total_angle = angle_max - angle_min;
	int num_of_rays = round(total_angle / angle_increment); // 1000
	
	// get shortest dist index
	int shortestDistIdx = 0;
	for(int i = 0; i < num_of_rays; i++){
		if(_msg->ranges[shortestDistIdx] > _msg->ranges[i]){
			shortestDistIdx = i;
		}
	}

	// group objects in lidar fov to simulate mmwave readings
	std::vector<float> object_center_angls;
	std::vector<float> object_center_dists;
	int grouped_previous = 0;
	float group_dist = 0;
	float group_angl = 0;
	for(int i = 0; i < num_of_rays-1; i++){
		if(_msg->ranges[i] > range_min && _msg->ranges[i] < range_max){
			//std::cout << "Object detected, range: " << _msg->ranges[i] << std::endl;
			if(grouped_previous == 0){	
				//std::cout << "First beam of object" << std::endl;
				group_dist += _msg->ranges[i];
				group_angl += float(i)*angle_increment - angle_max;
			}
			if( abs(_msg->ranges[i+1] - _msg->ranges[i]) < 0.1 ){
				//std::cout << "Object more than one beam wide" << std::endl;
				group_dist += _msg->ranges[i+1];
				group_angl += float(i+1)*angle_increment - angle_max;
				grouped_previous++;
			}
			else{
				//std::cout << "End of object detected, pushing" << std::endl;
				object_center_dists.push_back( group_dist/(grouped_previous+1) );
				object_center_angls.push_back( group_angl/(grouped_previous+1) );
				grouped_previous = 0;
				group_dist = 0;
				group_angl = 0;
			}
		}
	}
	this->objects_angl = object_center_angls;
	this->objects_dist = object_center_dists;
	
	// angle compared to straight up from drone
	float shortestDistIdxAngle = float(shortestDistIdx)*angle_increment - angle_max; 
	std::cout << "angle: " << shortestDistIdxAngle << std::endl;
	RCLCPP_INFO(this->get_logger(),  "Dist: %f, Angle: %f'", _msg->ranges[shortestDistIdx], shortestDistIdxAngle);

	this->shortestDist_ = _msg->ranges[shortestDistIdx];
	this->shortestDistAngle_ = shortestDistIdxAngle;

	// convert from "spherical" to cartesian
	std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;
	for(int i = 0; i<objects_dist.size(); i++){
		pcl_x.push_back( sin(object_center_angls.at(i)) * object_center_dists.at(i) );
		pcl_y.push_back( sin(0) * object_center_dists.at(i) );
		pcl_z.push_back( cos(object_center_angls.at(i)) * object_center_dists.at(i) );
	}

	// create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	pcl2_msg.header = std_msgs::msg::Header();
	pcl2_msg.header.stamp = this->now();
	std::string frameID = "map";
	pcl2_msg.header.frame_id = frameID;
	pcl2_msg.fields.resize(3);
	pcl2_msg.fields[0].name = 'x';
	pcl2_msg.fields[0].offset = 0;
	pcl2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[0].count = 1;
	pcl2_msg.fields[1].name = 'y';
	pcl2_msg.fields[1].offset = 4;
	pcl2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[1].count = 1;
	pcl2_msg.fields[2].name = 'z';
	pcl2_msg.fields[2].offset = 8;
	pcl2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl2_msg.fields[2].count = 1;
	const uint32_t POINT_STEP = 12;
	pcl2_msg.data.resize(std::max((size_t)1, objects_dist.size()) * POINT_STEP, 0x00);
	pcl2_msg.point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl2_msg.row_step = pcl2_msg.data.size();//pcl2_msg.point_step * pcl2_msg.width; // only 1 row because unordered
	pcl2_msg.height = 1;  // because unordered cloud
	pcl2_msg.width = pcl2_msg.row_step / POINT_STEP; // number of points in cloud
	pcl2_msg.is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	uint8_t *ptr = pcl2_msg.data.data();
	for (size_t i = 0; i < objects_dist.size(); i++)
	{
		*(reinterpret_cast<float*>(ptr + 0)) = pcl_x.at(i);
		*(reinterpret_cast<float*>(ptr + 4)) = pcl_y.at(i);
		*(reinterpret_cast<float*>(ptr + 8)) = pcl_z.at(i);
		ptr += POINT_STEP;
	}
	// publish PointCloud2 msg
	this->lidar_to_mmwave_pcl_publisher_->publish(pcl2_msg);
}


	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting vel_ctrl_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityControlVectorAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
