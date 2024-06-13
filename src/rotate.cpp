#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <robotcontrol.h>

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
#define MOTOR_POLARITY -1

using namespace std::chrono_literals;

class RotateNode : public rclcpp::Node
{
public:
    RotateNode() : Node("rotate")
    {

	timer_ = this->create_wall_timer(10ms, std::bind(&RotateNode::control_loop, this));

	// Declare and initialize PID controller parameters
        this->declare_parameter<double>("Kp", 0.0012); //0.0015
        this->declare_parameter<double>("Ki", 0.00015); //0.00015
        this->declare_parameter<double>("Kd", 0.0); //0.0
	this->declare_parameter<double>("theta", 90.0);
        this->get_parameter("Kp", Kp_);
        this->get_parameter("Ki", Ki_);
        this->get_parameter("Kd", Kd_);
	this->get_parameter("theta", rotate_angle_);

	parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RotateNode::parameter_callback, this, std::placeholders::_1)
        );

	//Initialize Derivative and Integral Variables
	prev_error_ = 0;
	integral_ = 0;

	//Initialize Robot Control Library
	if (rc_initialize() != 0) {
	    RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
	    rclcpp::shutdown();
	}

	// Initialize IMU using RCL
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	conf.enable_magnetometer = 1;
	conf.dmp_sample_rate = 100;
        // Initialize the magnetometer
        if (rc_mpu_initialize_dmp(&mpu_data_, conf) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize MPU");
            rclcpp::shutdown();
        }

	rc_motor_free_spin(1);
    }

    ~RotateNode()
    {
	//Cleanup
	rc_mpu_power_off();
	rc_cleanup();
    }

private:
    void control_loop()
    {
	//Get heading from MPU
	double heading = mpu_data_.compass_heading;
	heading = 180 * heading / 3.141592 + 90;

	//Compute Control Signal using PID
	double error = rotate_angle_ - heading;
	integral_ += error*0.01;
	double derivative = error - prev_error_;
	double control_signal = std::max(-0.65, std::min(0.65, Kp_ * error + Ki_ * integral_ + Kd_ * derivative));
	prev_error_ = error;

	//Send control signal to motor
	rc_motor_set(1, MOTOR_POLARITY * control_signal);

	//Print Values
	RCLCPP_INFO(this->get_logger(), "Ref: %.2f, Head: %.2f, Err: %.2f, Ctrl: %.4f, Dirv: %.4f, Intg: %.4f, Kp: %.4f, Ki: %.4f, Kd: %.4f",
                    rotate_angle_, heading, error, control_signal, derivative, integral_, Kp_, Ki_, Kd_);
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
		rotate_angle_ = param.as_double();
	    }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }


    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rc_mpu_data_t mpu_data_;
    double rotate_angle_;
    double Kp_, Ki_, Kd_;
    double prev_error_;
    double integral_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotateNode>());
    rclcpp::shutdown();
    return 0;
}


