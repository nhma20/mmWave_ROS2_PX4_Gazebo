#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>
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
						
			lidar_to_mmwave_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/lidar_to_mmwave_pcl",	10,
			std::bind(&VelocityControlVectorAdvertiser::OnDepthMsg, this, std::placeholders::_1));
			
		}

		~VelocityControlVectorAdvertiser() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down vel_ctrl_vect_advertiser..");
		}


	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_to_mmwave_pcl_subscription_;

		int callback_count = 0;
		void OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
		void VelocityDroneControl(float xv, float yv, float zv);
		float constrain(float val, float lo_lim, float hi_lim);
		bool beginHover();
};


// publish drone velocity vector
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
	// find closest point in pointcloud msg
	for (int i = 0; i < pcl_size; i++)
	{
		current_dist = sqrt( pow(pcl_x.at(i), 2) + pow(pcl_y.at(i), 2) + pow(pcl_z.at(i), 2) );
		if( current_dist < closest_dist ){
			closest_dist = current_dist;
			closest_dist_idx = i;
		}
	}

	static float shortest_dist_angle_xz;
	static float shortest_dist_angle_yz;
	static float shortest_dist;
	float prev_shortest_dist_angle_xz;
	float prev_shortest_dist_angle_yz;
	float prev_shortest_dist;

	if(pcl_size > 0){
		prev_shortest_dist_angle_xz = shortest_dist_angle_xz;
		prev_shortest_dist_angle_yz = shortest_dist_angle_yz;
		prev_shortest_dist = shortest_dist;
		shortest_dist_angle_xz = asin(pcl_x.at(closest_dist_idx) / sqrt(pow(pcl_x.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		shortest_dist_angle_yz = asin(pcl_y.at(closest_dist_idx) / sqrt(pow(pcl_y.at(closest_dist_idx),2) + pow(pcl_z.at(closest_dist_idx),2)));
		shortest_dist = closest_dist;
		RCLCPP_INFO(this->get_logger(),  "\n Dist: %f, \n XZ Angle: %f, \n YX Angle: %f", shortest_dist, shortest_dist_angle_xz, shortest_dist_angle_yz);

	} else{	
		RCLCPP_INFO(this->get_logger(),  "\n No points in pointcloud");
	}

	auto vel_ctrl_vect = px4_msgs::msg::DebugVect();
	vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
	std::string name = "test";
	std::copy(name.begin(), name.end(), vel_ctrl_vect.name.begin());
	
	bool hovering = true; //beginHover();

	float control_distance = 0.5; // desired distance to cable (meters)
	float control_angle = 0.0; // desired angle to cable (rad)
	float kp_dist = 0.5; // proportional gain for distance controller - nonoise 1.5
	float kp_angle = 5.0; // proportional gain for angle controller - nonoise 5.0
	float kd_dist = 0.015; // derivative gain for distance controller - nonoise 1.5
	float kd_angle = 0.0005; // derivative gain for angle controller - nonoise 0.5
	float ki_dist = 0.01; // integral gain for distance controller - nonoise 0.05
	float ki_angle = 0.01; // integral gain for angle controller - nonoise 0.05

	/*auto time
	static auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;*/

	float dt = 0.02; // seconds
	float d_dist = ((shortest_dist-control_distance) - (prev_shortest_dist-control_distance)) / dt;
	float d_angle_xz = ((shortest_dist_angle_xz-control_angle) - (prev_shortest_dist_angle_xz-control_angle)) / dt;
	float d_angle_yz = ((shortest_dist_angle_yz-control_angle) - (prev_shortest_dist_angle_yz-control_angle)) / dt;

	static float i_dist;
	static float i_angle_xz;
	static float i_angle_yz;
	i_dist += (shortest_dist-control_distance) * dt;
	// reset integral terms when error crosses 0
	if( (i_dist < 0 && ((shortest_dist-control_distance) * dt > 0)) || (i_dist > 0 && ((shortest_dist-control_distance) * dt < 0)) ){
		i_dist = 0;
	}
	i_angle_xz += (shortest_dist_angle_xz-control_angle) * dt;
	if( (i_angle_xz < 0 && ((shortest_dist_angle_xz-control_angle)* dt > 0)) || (i_angle_xz > 0 && ((shortest_dist_angle_xz-control_angle) * dt < 0)) ){
		i_angle_xz = 0;
	}
	i_angle_yz += (shortest_dist_angle_yz-control_angle) * dt;
	if( (i_angle_yz < 0 && ((shortest_dist_angle_yz-control_angle)* dt > 0)) || (i_angle_yz > 0 && ((shortest_dist_angle_yz-control_angle) * dt < 0)) ){
		i_angle_yz = 0;
	}

	if(pcl_size > 0){
		std::cout << "Points: " << pcl_size << std::endl;
		// create velocity control vector to steer drone towards cable
		VelocityDroneControl(constrain(kp_angle*(shortest_dist_angle_yz-control_angle) + kd_angle*d_angle_yz + ki_angle*i_angle_yz,-0.75,0.75), 
							constrain(kp_angle*(shortest_dist_angle_xz-control_angle) + kd_angle*d_angle_xz + ki_angle*i_angle_xz,-3,3), 
							constrain(-kp_dist*(shortest_dist-control_distance) + (-kd_dist*d_dist) + (-ki_dist*i_dist),-1,1));
		//VelocityDroneControl(0, kp_angle*(shortest_dist_angle_xz-control_angle), -kp_dist*(shortest_dist-control_distance));
	} else {	
		// control drone in square motion if nothing within lidar fov
		static int callbackscale = 5;
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


// constrains value to be between limits
float VelocityControlVectorAdvertiser::constrain(float val, float lo_lim, float hi_lim){
	if (val < lo_lim)
	{
		return lo_lim;
	}
	else if (val > hi_lim)
	{
		return hi_lim;
	}
	else{
		return val;
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


	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting velocity control vector advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityControlVectorAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
