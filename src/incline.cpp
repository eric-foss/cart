#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robotcontrol.h>

using namespace std::chrono_literals;

class InclineNode : public rclcpp::Node
{
public:
    InclineNode()
        : Node("incline"), motor_running_(false), theta_(0), tilt_counter_(0), no_hitch_counter_(0)
    {
        //Set up motor subscriber: runs motor when 1, stops motor when 0
        motor_status_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "motor_status", 10, std::bind(&InclineNode::start_motor_callback, this, std::placeholders::_1));

        //Set up cart status publisher: wants to rotate when 1, doesn't when 0
        cart_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("cart_status", 10);
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        cart_status_publisher_->publish(msg);

        //Set up vesc publisher
        vesc_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&InclineNode::control_loop, this));

        status_timer_ = this->create_wall_timer(
            5000ms, std::bind(&InclineNode::status_loop, this));

        // Initialize robot control library
        if (rc_initialize() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize robot control library");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "RC Library has been initialized");

	    // Initialize Motors
        if (rc_motor_init() != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize motor");
            rclcpp::shutdown();
        }

        // Initialize IMU
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = 2;
        if (rc_mpu_initialize(&data_, conf) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize MPU");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "IMU has been initialized");

        // Set motor to freewheel at the start
        rc_motor_free_spin(1);

        cart_status_publisher_->publish(msg);
    }

    ~InclineNode()
    {
        // Cleanup robot control library
        rc_motor_free_spin(1);
        rc_mpu_power_off();
        rc_cleanup();
    }

private:
    void start_motor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !motor_running_) {
            RCLCPP_INFO(this->get_logger(), "Starting motor");
            motor_running_ = true;
            rc_motor_set(1, 0.5);  // Set motor to half speed
        }
    }

    void control_loop()
    {
	    if (rc_mpu_read_accel(&data_) != 0) {
	        RCLCPP_WARN(this->get_logger(), "Failed to read accelerometer data");
	        return;
	    }

	    double x = data_.accel[0];
	    double y = data_.accel[1];
	    double z = data_.accel[2];
	    theta_ = atan(y/z);


        if (motor_running_) {

            no_hitch_counter_ ++;

	        if (theta_ < -0.08) {
		        tilt_counter_ ++;
	        }
	        else {
		        tilt_counter_ = 0;
	        }

            if (tilt_counter_ > 5) {  // Threshold for significant tilt
                RCLCPP_INFO(this->get_logger(), "Significant tilt detected. Stopping motor.");
                rc_motor_free_spin(1);
                motor_running_ = false;
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                cart_status_publisher_->publish(msg);
            }

            if (no_hitch_counter_ > 120) {
                auto vesc_msg_1 = geometry_msgs::msg::Twist();
                vesc_msg_1.linear.x = -0.5;
                vesc_publisher_->publish(vesc_msg_1);
                vesc_timer_ = this->create_wall_timer(
                    100ms, std::bind(&InclineNode::vesc_stop, this));
                no_hitch_counter_ = 0;
            }


        }

	    //Status Logger
	    //RCLCPP_INFO(this->get_logger(), "Status: %d, x_accel: %.3f, y_accel: %.3f, z_accel: %.3f, Tilt: %.3f, Counter: %d", motor_running_, x, y, z, theta_, counter_);

    }

    void vesc_stop()
    {
        auto vesc_msg_2 = geometry_msgs::msg::Twist();
        vesc_msg_2.linear.x = 0.0;
        vesc_publisher_->publish(vesc_msg_2);
        vesc_timer_->cancel();

    }

    void status_loop()
    {

	RCLCPP_INFO(this->get_logger(), "Incline Node Running");

    }


    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_status_subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cart_status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vesc_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr vesc_timer_;
    rc_mpu_data_t data_;
    bool motor_running_;
    double theta_;
    int32_t tilt_counter_;
    int32_t no_hitch_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InclineNode>());
    rclcpp::shutdown();
    return 0;
}
