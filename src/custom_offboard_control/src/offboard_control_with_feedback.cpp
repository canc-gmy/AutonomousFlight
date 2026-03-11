#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;

class OffboardSquareMission : public rclcpp::Node
{
public:
    OffboardSquareMission() : Node("offboard_square_mission")
    {
        offboard_pub_ = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        cmd_pub_ = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&OffboardSquareMission::local_position_cb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms,
            std::bind(&OffboardSquareMission::timer_cb, this));

        init_waypoints();

        RCLCPP_INFO(get_logger(), "Offboard square mission node started");
    }

private:
    enum class MissionState
    {
        WAIT_OFFBOARD,
        TAKEOFF,
        WAYPOINT,
        LAND,
        DONE
    };

    struct Waypoint
    {
        float x;
        float y;
        float z;
    };

    /* ROS entities */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;

    /* State */
    MissionState state_{MissionState::WAIT_OFFBOARD};
    uint64_t setpoint_counter_{0};
    size_t waypoint_index_{0};

    /* Feedback */
    float x_{0.f}, y_{0.f}, z_{0.f};

    /* Mission */
    std::vector<Waypoint> waypoints_;
    const float waypoint_tolerance_{0.3f};

    /* ---------------- Callbacks ---------------- */

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;
    }

    void timer_cb()
    {
        publish_offboard_mode();

        if (setpoint_counter_ < 10) {
            setpoint_counter_++;
            publish_setpoint(0.f, 0.f, -2.f);
            return;
        }

        if (setpoint_counter_ == 10) {
            setpoint_counter_++;
            enable_offboard();
            arm();
            state_ = MissionState::TAKEOFF;
        }

        run_mission();
    }

    /* ---------------- Mission logic ---------------- */

    void run_mission()
    {
        switch (state_) {

        case MissionState::TAKEOFF:
            publish_setpoint(0.f, 0.f, -2.f);
            if (reached(0.f, 0.f, -2.f)) {
                state_ = MissionState::WAYPOINT;
                waypoint_index_ = 0;
                RCLCPP_INFO(get_logger(), "Takeoff complete");
            }
            break;

        case MissionState::WAYPOINT:
            if (waypoint_index_ >= waypoints_.size()) {
                state_ = MissionState::LAND;
                break;
            }

            publish_setpoint(
                waypoints_[waypoint_index_].x,
                waypoints_[waypoint_index_].y,
                waypoints_[waypoint_index_].z);

            if (reached(waypoints_[waypoint_index_])) {
                RCLCPP_INFO(get_logger(), "Waypoint %zu reached", waypoint_index_);
                waypoint_index_++;
            }
            break;

        case MissionState::LAND:
            land();
            state_ = MissionState::DONE;
            break;

        case MissionState::DONE:
            break;

        default:
            break;
        }
    }

    /* ---------------- Helpers ---------------- */

    void init_waypoints()
    {
        waypoints_ = {
            {1.f, 0.f, -2.f},
            {1.f, 1.f, -2.f},
            {0.f, 1.f, -2.f},
            {0.f, 0.f, -2.f}
        };
    }

    bool reached(const Waypoint &wp)
    {
        return reached(wp.x, wp.y, wp.z);
    }

    bool reached(float tx, float ty, float tz)
    {
        float dx = tx - x_;
        float dy = ty - y_;
        float dz = tz - z_;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < waypoint_tolerance_;
    }

    void publish_offboard_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = now_us();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z)
    {
        TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = 0.f;
        msg.timestamp = now_us();
        traj_pub_->publish(msg);
    }

    void enable_offboard()
    {
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
    }

    void land()
    {
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(get_logger(), "Landing");
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f)
    {
        VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = now_us();
        cmd_pub_->publish(msg);
    }

    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardSquareMission>());
    rclcpp::shutdown();
    return 0;
}
