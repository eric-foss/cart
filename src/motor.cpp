#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <robotcontrol.h>

using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node
{
public:
    MotorNode() : Node("motor")
    {
        reference_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "reference_angle", 10,
            std::bind(&MotorNode::reference_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            10ms, std::bind(&MotorNode::control_loop, this));
        
        // PID controller parameters
        Kp_ = 0.001;  // Proportional gain
        Ki_ = 0.0; // Integral gain
        Kd_ = 0.0; // Derivative gain
        prev_error_ = 0;
        integral_ = 0;
	reference_angle_ = 60.0;

        // Initialize robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }

        // Set motor to freewheel at the start
        rc_motor_free_spin(1);
    }

    ~MotorNode()
    {
        // Cleanup robot control library
        rc_motor_free_spin(1);
        rc_cleanup();
    }

private:
    void reference_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        reference_angle_ = msg->data;
    }

    void control_loop()
    {
        int32_t actual_encoder_value = rc_encoder_read(1);
	double actual_wheel_angle = actual_encoder_value * 3.141592 / 1250;
	double reference_wheel_angle = reference_angle_;
        double error = reference_wheel_angle - actual_wheel_angle;

        // PID control calculations
        integral_ += error;
        double derivative = error - prev_error_;
        double control_signal = std::max(-1.0, std::min(1.0, Kp_ * error + Ki_ * integral_ + Kd_ * derivative));
        // Send control signal to motor (ensure the value is within motor limits)
        rc_motor_set(1, control_signal);

        // Log the values (optional)
        RCLCPP_INFO(this->get_logger(), "Ref: %d, Act: %d, Err: %d, Ctrl: %d",
                    reference_wheel_angle, actual_wheel_angle, error, control_signal);

        prev_error_ = error;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double reference_angle_;
    double Kp_, Ki_, Kd_;
    double prev_error_;
    double integral_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
