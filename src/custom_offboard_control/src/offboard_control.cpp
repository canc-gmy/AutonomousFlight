#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class CustomOffboardControl : public rclcpp::Node
{
    public:
    CustomOffboardControl() : Node("custom_offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        offboard_setpoint_counter_ = 0;
        mission_state_ = 0; // 0: takeoff, 1: left, 2: forward, 3: right, 4: back, 5: land
        state_counter_ = 0;
        
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                // Arm the vehicle
                this->arm();
            }
            
            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            
            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }
    
    void arm();
    void disarm();
    
    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;
    int mission_state_;
    int state_counter_;
    
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void CustomOffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void CustomOffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void CustomOffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void CustomOffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    
    // Only start mission after offboard mode is established
    if (offboard_setpoint_counter_ > 10) {
        state_counter_++;
        
        // Each state lasts for 50 cycles (5 seconds at 100ms intervals)
        if (state_counter_ >= 50 && mission_state_ < 5) {
            state_counter_ = 0;
            mission_state_++;
            RCLCPP_INFO(this->get_logger(), "Moving to mission state: %d", mission_state_);
        }
    }
    
    switch (mission_state_) {
        case 0: // Takeoff and hover at starting position
        msg.position = {0.0, 0.0, -5.0};
        msg.yaw = 0.0;
        break;
        case 1: // Move 1m to the left
        msg.position = {0.0, 1.0, -5.0};
        msg.yaw = 0.0;
        break;
        case 2: // Move 1m forward
        msg.position = {1.0, 1.0, -5.0};
        msg.yaw = 0.0;
        break;
        case 3: // Move 1m to the right
        msg.position = {1.0, 0.0, -5.0};
        msg.yaw = 0.0;
        break;
        case 4: // Move 1m back (return to start position)
        msg.position = {0.0, 0.0, -5.0};
        msg.yaw = 0.0;
        break;
        case 5: // Land
        msg.position = {0.0, 0.0, 0.0};
        msg.yaw = 0.0;
        if (state_counter_ == 1) { // Send land command once
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            RCLCPP_INFO(this->get_logger(), "Landing command sent");
        }
        break;
        default: // Mission complete, maintain last position
        msg.position = {0.0, 0.0, 0.0};
        msg.yaw = 0.0;
        break;
    }
    
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void CustomOffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting custom offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomOffboardControl>());
    
    rclcpp::shutdown();
    return 0;
}
