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

	// Declare and initialize PID controller parameters
        this->declare_parameter<double>("Kp", 0.0012); //0.0015
        this->declare_parameter<double>("Ki", 0.00015); //0.00015
        this->declare_parameter<double>("Kd", 0.0); //0.0
	this->declare_parameter<double>("theta", 90.0);
        this->get_parameter("Kp", Kp_);
        this->get_parameter("Ki", Ki_);
        this->get_parameter("Kd", Kd_);
	this->get_parameter("theta", reference_angle_);

        prev_error_ = 0;
        integral_ = 0;

        // Initialize robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }

        // Set motor to freewheel at the start
        rc_motor_free_spin(1);

	// Set up parameter change callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MotorNode::parameter_callback, this, std::placeholders::_1)
        );

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
	double actual_wheel_angle = actual_encoder_value * 180 / 1250;
	double reference_wheel_angle = reference_angle_ * 330/90;
        double error = reference_wheel_angle - actual_wheel_angle;

        // PID control calculations
        integral_ += error*0.01;
        double derivative = error - prev_error_;
        double control_signal = std::max(-0.65, std::min(0.65, Kp_ * error + Ki_ * integral_ + Kd_ * derivative));
        // Send control signal to motor (ensure the value is within motor limits)
        rc_motor_set(1, control_signal);

        // Log the values (optional)
        RCLCPP_INFO(this->get_logger(), "Ref: %.2f, Whl: %.2f, Err: %.2f, Ctrl: %.4f, Dirv: %.4f, Intg: %.4f, Kp: %.4f, Ki: %.4f, Kd: %.4f",
                    reference_wheel_angle, actual_wheel_angle, error, control_signal, derivative, integral_, Kp_, Ki_, Kd_);

        prev_error_ = error;
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters) {
            if (param.get_name() == "Kp") {
                Kp_ = param.as_double();
            } else if (param.get_name() == "Ki") {
                Ki_ = param.as_double();
            } else if (param.get_name() == "Kd") {
                Kd_ = param.as_double();
            } else if (param.get_name() == "theta") {
		reference_angle_ = param.as_double();
	    }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
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
