#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robotcontrol.h>

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
#define MOTOR_POLARITY -1

using namespace std::chrono_literals;

class MagnetometerNode : public rclcpp::Node
{
public:
    MagnetometerNode() : Node("magnetometer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node Set up Starting ...");
        // Set up control loop timer
        control_timer_ = this->create_wall_timer(20ms, std::bind(&MagnetometerNode::control_loop, this));

        //Set up vesc publisher
        vesc_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        successful_latch_publisher_ = this->create_publisher<std_msgs::msg::Bool>("successful_latch", 10);

        //Set up tuning parameters
        this->declare_parameter<double>("Kp", 0.005); //0.0015
        this->declare_parameter<double>("Ki", 0.0005); //0.00015
        this->declare_parameter<double>("Kd", 0.0); //0.0
        this->get_parameter("Kp", Kp_);
        this->get_parameter("Ki", Ki_);
        this->get_parameter("Kd", Kd_);

        desired_angle_ = 0.0;
        prev_error_ = 0.0;
        tilt_counter_ = 0;

        // Set up parameter change callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MagnetometerNode::parameter_callback, this, std::placeholders::_1));


        // Initialize the robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Robot Control Library Initialized");

        // Initialize dmp IMU
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
        RCLCPP_INFO(this->get_logger(), "MPU DMP Initialized");
        // Initialize motors
        // Initialize Motors
        if (rc_motor_init() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize motor");
            rclcpp::shutdown();
        }
        rc_motor_free_spin(1);
        RCLCPP_INFO(this->get_logger(), "Motor Initialized");

        RCLCPP_INFO(this->get_logger(), "Magnetometer node has been initialized");
    }

    ~MagnetometerNode()
    {
        // Cleanup the robot control library
        rc_motor_free_spin(1);
        rc_mpu_power_off();
        rc_cleanup();
    }

private:
    void control_loop()
    {

        // Extract magnetometer data
        double mag_x = mpu_data_.mag[0];
        double mag_y = mpu_data_.mag[1];
	    double mag_z = mpu_data_.mag[2];
	    heading_ = mpu_data_.compass_heading;

        // Extract accelerometer data
        double accel_x = mpu_data_.accel[0];
        double accel_y = mpu_data_.accel[1];
        double accel_z = mpu_data_.accel[2];
        theta_ = atan(accel_y/accel_z);

        // Compute the angle of the y-axis with respect to north
        double angle = atan2(mag_x, mag_y);
	    angle = 180 * angle / 3.141592;

	    // Convert Filtered Heading to y axis and compute error
	    heading_ = 180 * heading_ / 3.141592;
	    heading_ = heading_ + 90.0;
        error_ = desired_angle_ - heading_;

        integral_ += 0.01*error_;
        double derivative = error_ - prev_error_;
        double control_signal = std::max(-0.65, std::min(0.65, Kp_ * error_ + Ki_ * integral_ + Kd_ * derivative));
	    prev_error_ = error_;

        //RCLCPP_INFO(this->get_logger(), "Angle: %.2f, Filtered Heading: %.2f", angle, heading);
        // Check if latched
        if (theta_ < -0.08) {
            tilt_counter_ ++;
        } else {
            tilt_counter_ = 0;
        }

        if (tilt_counter_ > 5) {  // Threshold for significant tilt
            RCLCPP_INFO(this->get_logger(), "Significant tilt detected. Stopping motor.");
            rc_motor_free_spin(1);
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            successful_latch_publisher_->publish(msg);
            control_timer_->cancel();
        }

        //Send control signal to motor
	    rc_motor_set(1, MOTOR_POLARITY * control_signal);



        //Print Values
	    RCLCPP_INFO(this->get_logger(), "Ref: %.2f, Head: %.2f, Err: %.2f, Ctrl: %.4f, Dirv: %.4f, Intg: %.4f, Kp: %.4f, Ki: %.4f, Kd: %.4f",
                desired_angle_, heading_, error_, control_signal, derivative, integral_, Kp_, Ki_, Kd_);


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
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr successful_latch_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vesc_publisher_;
    rc_mpu_data_t mpu_data_;
    double desired_angle_;
    double error_;
    double theta_;
    double heading_;
    double Kp_, Ki_, Kd_;
    double prev_error_;
    double integral_;
    int32_t tilt_counter_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerNode>());
    rclcpp::shutdown();
    return 0;
}
