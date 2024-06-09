#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <robotcontrol.h>

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21

using namespace std::chrono_literals;

class MagnetometerNode : public rclcpp::Node
{
public:
    MagnetometerNode() : Node("magnetometer_node")
    {
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("magnetometer_angle", 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&MagnetometerNode::read_magnetometer_data, this));

        // Initialize the robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }
	
	

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


        RCLCPP_INFO(this->get_logger(), "Magnetometer node has been initialized");
    }

    ~MagnetometerNode()
    {
        // Cleanup the robot control library
        rc_mpu_power_off();
        rc_cleanup();
    }

private:
    void read_magnetometer_data()
    {
        //if (rc_mpu_read_mag(&mpu_data_) != 0) {
        //    RCLCPP_WARN(this->get_logger(), "Failed to read magnetometer data");
        //    return;
        //}

        // Extract magnetometer data
        double mag_x = mpu_data_.mag[0];
        double mag_y = mpu_data_.mag[1];
	double mag_z = mpu_data_.mag[2];
	double heading = mpu_data_.compass_heading;

        // Compute the angle of the y-axis with respect to north
        double angle = atan2(mag_x, mag_y);
	angle = 180 * angle / 3.141592;

	// Convert Filtered Heading to y axis
	heading = 180 * heading / 3.141592;
	heading = heading + 90.0;

        // Publish the angle
        auto message = std_msgs::msg::Float64();
        message.data = angle;
        angle_publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Angle: %.2f, Filtered Heading: %.2f", angle, heading);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rc_mpu_data_t mpu_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerNode>());
    rclcpp::shutdown();
    return 0;
}
