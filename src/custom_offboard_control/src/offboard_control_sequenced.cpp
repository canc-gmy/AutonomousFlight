#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ==============================================================================
// NODO 0: Ground Vehicle Simulator (Lifecycle)
// ==============================================================================
class GroundVehicleSimulator : public rclcpp_lifecycle::LifecycleNode
{
public:
    GroundVehicleSimulator() : LifecycleNode("ground_vehicle_simulator"),
        rng_(std::random_device{}()),
        vel_dist_(-1.0f, 1.0f),
        dur_dist_(1.0f, 4.0f)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        // Setup Publisher
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10);

        RCLCPP_INFO(get_logger(), "Configured: Resources allocated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        // Enable Publisher
        pose_pub_->on_activate();

        // Start Timer (Logic starts only when active)
        move_timer_ = create_wall_timer(50ms,
            std::bind(&GroundVehicleSimulator::update_position, this));

        pick_new_velocity();
        
        RCLCPP_INFO(get_logger(), "Activated: Simulation running. Pos: (%.1f, %.1f)", x_, y_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        pose_pub_->on_deactivate();
        move_timer_.reset(); // Stop timer
        if(dir_timer_) dir_timer_.reset();
        
        RCLCPP_INFO(get_logger(), "Deactivated: Simulation paused.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        pose_pub_.reset();
        RCLCPP_INFO(get_logger(), "Cleaned up: Resources released.");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    rclcpp::TimerBase::SharedPtr dir_timer_;

    std::mt19937 rng_;
    std::uniform_real_distribution<float> vel_dist_;
    std::uniform_real_distribution<float> dur_dist_;

    float x_{1.f}, y_{0.f};
    float vx_{0.f}, vy_{0.f};
    float target_vx_{0.f}, target_vy_{0.f};

    void pick_new_velocity()
    {
        // Polar coordinates for uniform direction distribution
        std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * M_PI);
        std::uniform_real_distribution<float> speed_dist(0.5f, 1.5f);

        float angle = angle_dist(rng_);
        float speed = speed_dist(rng_);

        target_vx_ = speed * std::cos(angle);
        target_vy_ = speed * std::sin(angle);

        float duration_s = dur_dist_(rng_);
        dir_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(duration_s * 1000)),
            [this]() {
                pick_new_velocity();
            });
    }

    void update_position()
    {
        constexpr float dt = 0.05f;

        // Smooth velocity transition (simple low-pass filter)
        vx_ += (target_vx_ - vx_) * 0.3f;
        vy_ += (target_vy_ - vy_) * 0.3f;

        x_ += vx_ * dt;
        y_ += vy_ * dt;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x_;
        msg.pose.position.y = y_;
        msg.pose.position.z = 0.1f;
        pose_pub_->publish(msg);
    }
};

// ==============================================================================
// NODO 1: Geofence Monitor (Lifecycle)
// ==============================================================================
class GeofenceMonitor : public rclcpp_lifecycle::LifecycleNode
{
public:
    GeofenceMonitor() : LifecycleNode("geofence_monitor") {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        abort_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&GeofenceMonitor::local_position_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Configured. Radius: %.1f m", GEOFENCE_RADIUS);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        abort_pub_->on_activate();
        RCLCPP_INFO(get_logger(), "Activated: Monitoring boundaries.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        abort_pub_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Deactivated: Monitoring paused.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        abort_pub_.reset();
        local_pos_sub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    static constexpr float GEOFENCE_RADIUS = 15.0f;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
    bool abort_triggered_{false};

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        // Only monitor if the node is in Active state
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        if (abort_triggered_) return;

        float dist = std::hypot(msg->x, msg->y);
        if (dist >= GEOFENCE_RADIUS) {
            RCLCPP_WARN(get_logger(), "Geofence Violation! dist=%.2f. Aborting!", dist);
            std_msgs::msg::Bool abort_msg;
            abort_msg.data = true;
            abort_pub_->publish(abort_msg);
            abort_triggered_ = true;
        }
    }
};

// ==============================================================================
// NODO 2: Offboard Mission (Lifecycle)
// ==============================================================================
class OffboardTrackingMission : public rclcpp_lifecycle::LifecycleNode
{
public:
    OffboardTrackingMission() : LifecycleNode("offboard_tracking_mission") {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_     = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_      = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        abort_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/abort", 10,
            std::bind(&OffboardTrackingMission::abort_cb, this, std::placeholders::_1));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&OffboardTrackingMission::local_position_cb, this, std::placeholders::_1));

        ground_vehicle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10,
            std::bind(&OffboardTrackingMission::ground_vehicle_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Configured. Ready to track.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        offboard_pub_->on_activate();
        traj_pub_->on_activate();
        cmd_pub_->on_activate();

        timer_ = create_wall_timer(100ms, std::bind(&OffboardTrackingMission::timer_cb, this));
        
        RCLCPP_INFO(get_logger(), "Activated: Mission Loop Started.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        offboard_pub_->on_deactivate();
        traj_pub_->on_deactivate();
        cmd_pub_->on_deactivate();
        timer_.reset();
        RCLCPP_INFO(get_logger(), "Deactivated: Mission Loop Stopped.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        offboard_pub_.reset();
        traj_pub_.reset();
        cmd_pub_.reset();
        local_pos_sub_.reset();
        ground_vehicle_sub_.reset();
        abort_sub_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    static constexpr float MIN_FOLLOW_DIST = 1.0f;
    static constexpr float HYSTERESIS      = 0.2f;
    static constexpr float CRUISE_ALT      = -2.0f;
    static constexpr float FOLLOW_STEP     = 0.8f;

    enum class MissionState { WAIT_OFFBOARD, TAKEOFF, TRACK, LAND, DONE };
    enum class FollowState { HOLD, FOLLOWING };

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp_lifecycle::LifecyclePublisher<VehicleCommand>::SharedPtr      cmd_pub_;
    
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr            local_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_vehicle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             abort_sub_;

    MissionState state_{MissionState::WAIT_OFFBOARD};
    FollowState  follow_state_{FollowState::HOLD};
    uint64_t     setpoint_counter_{0};

    float drone_x_{0.f}, drone_y_{0.f}, drone_z_{0.f};
    float vehicle_x_ned_{0.f}, vehicle_y_ned_{0.f};
    bool  vehicle_data_received_{false};
    float hold_x_{0.f}, hold_y_{0.f};

    void abort_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && state_ != MissionState::LAND && state_ != MissionState::DONE) {
            RCLCPP_WARN(get_logger(), "Abort signal received!");
            state_ = MissionState::LAND;
        }
    }

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        drone_x_ = msg->x; drone_y_ = msg->y; drone_z_ = msg->z;
    }

    void ground_vehicle_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vehicle_x_ned_ = static_cast<float>(msg->pose.position.y);
        vehicle_y_ned_ = static_cast<float>(msg->pose.position.x);
        vehicle_data_received_ = true;
    }

    void timer_cb()
    {
        publish_offboard_mode();

        if (setpoint_counter_ < 10) {
            setpoint_counter_++;
            publish_setpoint(0.f, 0.f, CRUISE_ALT);
            return;
        }

        if (setpoint_counter_ == 10 && state_ != MissionState::DONE && state_ != MissionState::LAND) {
            setpoint_counter_++;
            enable_offboard();
            arm();
            state_ = MissionState::TAKEOFF;
        }

        run_mission();
    }

    void run_mission()
    {
        switch (state_) {
        case MissionState::TAKEOFF:
            publish_setpoint(0.f, 0.f, CRUISE_ALT);
            if (reached(0.f, 0.f, CRUISE_ALT)) {
                hold_x_ = drone_x_; hold_y_ = drone_y_;
                state_  = MissionState::TRACK;
            }
            break;
        case MissionState::TRACK:
            if (!vehicle_data_received_) {
                publish_setpoint(hold_x_, hold_y_, CRUISE_ALT);
                break;
            }
            track_ground_vehicle();
            break;
        case MissionState::LAND:
            land();
            state_ = MissionState::DONE;
            break;
        case MissionState::DONE:
            if (reached(hold_x_, hold_y_, 0.f)) {
                disarm();
            }
            break;
        default: break;
        }
    }

    void track_ground_vehicle()
    {
        float dx = vehicle_x_ned_ - drone_x_;
        float dy = vehicle_y_ned_ - drone_y_;
        float dist = std::hypot(dx, dy);

        if (follow_state_ == FollowState::HOLD) {
            if (dist >= MIN_FOLLOW_DIST + HYSTERESIS) {
                follow_state_ = FollowState::FOLLOWING;
                RCLCPP_INFO(get_logger(), "State switched: HOLD -> FOLLOWING. Distance to vehicle: %.2f m", dist);
            }
        } else {
            if (dist < MIN_FOLLOW_DIST) {
                follow_state_ = FollowState::HOLD;
                hold_x_ = drone_x_; hold_y_ = drone_y_;
                RCLCPP_INFO(get_logger(), "State switched: FOLLOWING -> HOLD. Distance to vehicle: %.2f m", dist);
            }
        }

        if (follow_state_ == FollowState::HOLD) {
            publish_setpoint(hold_x_, hold_y_, CRUISE_ALT);
        } else {
            float nx = dx / dist; float ny = dy / dist;
            float step = std::min(FOLLOW_STEP, dist);
            publish_setpoint(drone_x_ + nx * step, drone_y_ + ny * step, CRUISE_ALT);
            hold_x_ = drone_x_; hold_y_ = drone_y_;
        }
    }

    bool reached(float tx, float ty, float tz, float tol = 0.3f)
    {
        return std::hypot(tx-drone_x_, ty-drone_y_, tz-drone_z_) < tol;
    }

    void publish_offboard_mode()
    {
        OffboardControlMode msg{};
        msg.position = true; msg.timestamp = now_us();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z)
    {
        TrajectorySetpoint msg{};
        msg.position = {x, y, z}; msg.yaw = 0.f; msg.timestamp = now_us();
        traj_pub_->publish(msg);
    }

    void enable_offboard() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f); }
    void arm() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f); }
    void disarm() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f); }
    void land() { publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND); }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f)
    {
        VehicleCommand msg{};
        msg.command = cmd; msg.param1 = p1; msg.param2 = p2;
        msg.target_system = 1; msg.target_component = 1;
        msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true; msg.timestamp = now_us();
        cmd_pub_->publish(msg);
    }

    uint64_t now_us() { return this->get_clock()->now().nanoseconds() / 1000; }
};

// ==============================================================================
// NODO 3: GazeboVisualizer (Wrapped in Lifecycle for consistency)
// ==============================================================================
class GazeboVisualizer : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit GazeboVisualizer(const std::string & world_name = "default")
    : LifecycleNode("gazebo_visualizer"), world_name_(world_name) {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        set_pose_service_ = "/world/" + world_name_ + "/set_pose";
        vehicle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10,
            std::bind(&GazeboVisualizer::vehicle_cb, this, std::placeholders::_1));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override { return CallbackReturn::SUCCESS; }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override { return CallbackReturn::SUCCESS; }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override { 
        vehicle_sub_.reset(); 
        return CallbackReturn::SUCCESS; 
    }

private:
    std::string world_name_;
    std::string set_pose_service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_sub_;
    gz::transport::Node gz_node_;

    void vehicle_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Simplistic check: if not active, don't update viz
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;

        gz::msgs::Pose req;
        req.set_name("ground_vehicle");
        req.mutable_position()->set_x(msg->pose.position.x);
        req.mutable_position()->set_y(msg->pose.position.y);
        req.mutable_position()->set_z(-0.10);
        req.mutable_orientation()->set_w(1.0);
        gz::msgs::Boolean rep;
        bool result = false;
        gz_node_.Request(set_pose_service_, req, 50u, rep, result);
    }
};

// ==============================================================================
// MAIN: Direct C++ Sequencing
// ==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string world_name = (argc > 1) ? argv[1] : "default";

    // Instantiate Lifecycle Nodes
    auto vehicle_node  = std::make_shared<GroundVehicleSimulator>();
    auto geofence_node = std::make_shared<GeofenceMonitor>();
    auto viz_node      = std::make_shared<GazeboVisualizer>(world_name);
    auto mission_node  = std::make_shared<OffboardTrackingMission>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(vehicle_node->get_node_base_interface());
    executor.add_node(geofence_node->get_node_base_interface());
    executor.add_node(viz_node->get_node_base_interface());
    executor.add_node(mission_node->get_node_base_interface());

    // --- SEQUENCING STARTUP ---
    RCLCPP_INFO(rclcpp::get_logger("Startup"), ">>> PHASE 1: Configuring Infrastructure...");
    vehicle_node->configure();
    geofence_node->configure();
    viz_node->configure();

    RCLCPP_INFO(rclcpp::get_logger("Startup"), ">>> PHASE 2: Activating Simulation & Safety...");
    vehicle_node->activate();
    geofence_node->activate();
    viz_node->activate();

    RCLCPP_INFO(rclcpp::get_logger("Startup"), ">>> PHASE 3: Initializing Mission Node...");
    mission_node->configure();
    
    RCLCPP_INFO(rclcpp::get_logger("Startup"), ">>> PHASE 4: Starting Mission...");
    mission_node->activate();

    // Spin
    executor.spin();

    rclcpp::shutdown();
    return 0;
}