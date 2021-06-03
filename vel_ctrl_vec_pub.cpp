#include <rclcpp/rclcpp.hpp>
//#include <stdint.h>
#include <px4_msgs/msg/debug_vect.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

//creates a VelocityControlVectorAdvertiser class that subclasses the generic rclcpp::Node base class.
class VelocityControlVectorAdvertiser : public rclcpp::Node
{

//The code below creates a function for when messages are to be sent. 
//Messages are sent based on a timed callback.
	public:
		VelocityControlVectorAdvertiser() : Node("vel_ctrl_vect_advertiser") {
			publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("vel_ctrl_vect_topic", 10);
			static int callback_count = 0;
			auto timer_callback =
			[this]()->void {
				auto vel_ctrl_vect = px4_msgs::msg::DebugVect();
				vel_ctrl_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
				std::string name = "test";
				std::copy(name.begin(), name.end(), vel_ctrl_vect.name.begin());
				if(callback_count < 100){
					vel_ctrl_vect.x = NAN;
					vel_ctrl_vect.y = NAN;
					vel_ctrl_vect.z = -3.0; // go up for 10s
				} else if(callback_count % 100 < 25){
					vel_ctrl_vect.x = 0;
					vel_ctrl_vect.y = 3; // go east
					vel_ctrl_vect.z = 0;
				} else if(callback_count % 100 < 50){
					vel_ctrl_vect.x = 0;
					vel_ctrl_vect.y = 0;
					vel_ctrl_vect.z = -3; // go up
				} else if(callback_count % 100 < 75){
					vel_ctrl_vect.x = 0;
					vel_ctrl_vect.y = -3; // go west
					vel_ctrl_vect.z = 0;
				} else if(callback_count % 100 > 74){
					vel_ctrl_vect.x = 0;
					vel_ctrl_vect.y = 0;
					vel_ctrl_vect.z = 3; // go down
				} else{
					vel_ctrl_vect.x = 0;
					vel_ctrl_vect.y = 0; // do nothing if bugged
					vel_ctrl_vect.z = 0;
				}	
				callback_count++;
					
					
				RCLCPP_INFO(this->get_logger(), "\033[97m Publishing vel_ctrl_vect: time: %llu xv: %f yv: %f zv: %f \033[0m",
		                        vel_ctrl_vect.timestamp, vel_ctrl_vect.x, vel_ctrl_vect.y, vel_ctrl_vect.z);
				this->publisher_->publish(vel_ctrl_vect);
			};
			timer_ = this->create_wall_timer(100ms, timer_callback);
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};


int main(int argc, char *argv[])
{
	std::cout << "Starting vel_ctrl_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityControlVectorAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
