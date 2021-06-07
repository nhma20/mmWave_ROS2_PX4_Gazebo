#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <iostream>


//creates a lidar_test_pub class that subclasses the generic rclcpp::Node base class.
class lidar_test_pub : public rclcpp::Node
{
	public:
		lidar_test_pub() : Node("vel_ctrl_vect_advertiser") {
			subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/dist_sensor/laser_scan",	10,
			std::bind(&lidar_test_pub::OnSensorMsg, this, std::placeholders::_1));
		}


	private:
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

		void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
			std::cout << "1st dist: " << _msg->ranges[0] << std::endl;
		}
};


int main(int argc, char *argv[])
{
	std::cout << "Starting lidar test publisher node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<lidar_test_pub>());

	rclcpp::shutdown();
	return 0;
}

