#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robotcontrol.h>

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

	// Initialize robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "RC Library has been initialized");

	if (rc_servo_init() != 0) {
	    RCLCPP_FATAL(this->get_logger(), "Failed to initialize servos");
	    rclcpp::shutdown();
	}
	RCLCPP_INFO(this->get_logger(), "Servos have been initialized");

    }

    ~ServoNode()
    {
	//Cleanup
	rc_servo_cleanup();
	rc_cleanup();
    }

private:
    void servo1_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
	if (msg->data) {
	    RCLCPP_INFO(this->get_logger(), "Servo 1 Dispensing");
	    rc_servo_send_pulse_normalized(1, 0.5);
	} else {
	    RCLCPP_INFO(this->get_logger(), "Servo 1 Retracting");
	    rc_servo_send_pulse_normalized(1, -1.5);
	}
    }

    void servo2_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Servo 2 Dispensing");
            rc_servo_send_pulse_normalized(2, -1.5); 
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Servo 2 Retracting");
            rc_servo_send_pulse_normalized(2, 1.5);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo1_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo2_subscriber_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoNode>());
    rclcpp::shutdown();
    return 0;
}






