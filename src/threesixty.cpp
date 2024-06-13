#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <robotcontrol.h>

using namespace std::chrono_literals;

class ThreeSixtyNode : public rclcpp::Node
{
public:
    ThreeSixtyNode()
        : Node("threesixty"), motor_running_(false), prev_encoder_value_(0), speed_(0)
    {
        start_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_motor", 10, std::bind(&ThreeSixtyNode::start_motor_callback, this, std::placeholders::_1));
        completion_publisher_ = this->create_publisher<std_msgs::msg::Empty>("operation_complete", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&ThreeSixtyNode::control_loop, this));

        // Initialize robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }

        // Set motor to freewheel at the start
        rc_motor_free_spin(1);
    }

    ~ThreeSixtyNode()
    {
        // Cleanup robot control library
        rc_motor_free_spin(1);
        rc_cleanup();
    }

private:
    void start_motor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !motor_running_) {
            RCLCPP_INFO(this->get_logger(), "Starting motor");
            motor_running_ = true;
            rc_motor_set(1, 0.5);  // Set motor to half speed
            prev_encoder_value_ = rc_encoder_read(1);
        }
    }

    void control_loop()
    {
        if (motor_running_) {
            int32_t current_encoder_value = rc_encoder_read(1);
            speed_ = current_encoder_value - prev_encoder_value_;

            if (speed_ > 135){
		counter_ ++;
            }

            if (counter_ > 5) {  // Threshold for significant speed change
                RCLCPP_INFO(this->get_logger(), "Significant speed change detected. Stopping motor.");
                rc_motor_free_spin(1);
                motor_running_ = false;
                auto msg = std_msgs::msg::Empty();
                completion_publisher_->publish(msg);
            }

            prev_encoder_value_ = current_encoder_value;
        }

	RCLCPP_INFO(this->get_logger(), "Status: %d, Speed: %d",
                    motor_running_, speed_);


    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr completion_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool motor_running_;
    int32_t prev_encoder_value_;
    int32_t speed_;
    int32_t counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThreeSixtyNode>());
    rclcpp::shutdown();
    return 0;
}
