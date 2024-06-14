#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robotcontrol.h>

using namespace std::chrono_literals;

class ServoNode : public rclcpp::Node
{
public:
    ServoNode()
	: Node("servo")
    {

	//Set up Servo Subscribers
	servo1_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
	    "servo1", 10, std::bind(&ServoNode::servo1_callback, this, std::placeholders::_1));
	servo2_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
	    "servo2", 10, std::bind(&ServoNode::servo2_callback, this, std::placeholders::_1));

	//Status Timer
	status_timer_ = this->create_wall_timer(5000ms, std::bind(&ServoNode::status_loop, this));

    }

    ~ServoNode()
    {
	//Cleanup
	rc_cleanup();
    }

private:
    void servo1_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
	if (msg->data) {
	    RCLCPP_INFO(this->get_logger(), "Servo 1 Dispensing");
	    system("sudo /home/ubuntu/ros2_foxy/src/cart/c_scripts/move_servo -c 1 -p 0.5");
	} else {
	    RCLCPP_INFO(this->get_logger(), "Servo 1 Retracting");
	    system("sudo /home/ubuntu/ros2_foxy/src/cart/c_scripts/move_servo -c 1 -p -1.5");

	}
    }

    void servo2_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Servo 2 Dispensing");
            system("sudo /home/ubuntu/ros2_foxy/src/cart/c_scripts/move_servo -c 2 -p -1.5");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Servo 2 Retracting");
            system("sudo /home/ubuntu/ros2_foxy/src/cart/c_scripts/move_servo -c 2 -p 1.5");
        }
    }

    void status_loop()
    {

	RCLCPP_INFO(this->get_logger(), "Servo Node Running");

    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo1_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo2_subscriber_;
    rclcpp::TimerBase::SharedPtr status_timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoNode>());
    rclcpp::shutdown();
    return 0;
}






