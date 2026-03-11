#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;

// ==============================================================================
// NODO 1: Geofence Monitor
// Si occupa solo di controllare la posizione e lanciare l'allarme se necessario.
// ==============================================================================
class GeofenceMonitor : public rclcpp::Node
{
public:
    GeofenceMonitor() : Node("geofence_monitor")
    {
        // Publisher per inviare il segnale di interruzione missione
        abort_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Sottoscrizione alla posizione del drone
        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&GeofenceMonitor::local_position_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Geofence Monitor avviato. Area controllata: X[%.1f, %.1f] Y[%.1f, %.1f]", 
                    landing_zone_.x_min, landing_zone_.x_max, landing_zone_.y_min, landing_zone_.y_max);
    }

private:
    struct RestrictedArea
    {
        float x_min;
        float x_max;
        float y_min;
        float y_max;
    };

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;

    // Definizione dell'area ristretta
    RestrictedArea landing_zone_ = {0.5f, 1.5f, 0.5f, 1.5f};
    bool abort_triggered_{false};

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        // Se non abbiamo ancora lanciato l'allarme, controlliamo la posizione
        if (!abort_triggered_) {
            if (is_inside_restricted_area(msg->x, msg->y)) {
                RCLCPP_WARN(get_logger(), 
                    "Allarme Geofence! Drone entrato in zona vietata (x: %.2f, y: %.2f). Invio comando di aborto...", 
                    msg->x, msg->y);
                
                std_msgs::msg::Bool abort_msg;
                abort_msg.data = true;
                abort_pub_->publish(abort_msg);
                
                abort_triggered_ = true; // Lancia l'allarme una sola volta
            }
        }
    }

    bool is_inside_restricted_area(float current_x, float current_y)
    {
        return (current_x >= landing_zone_.x_min && current_x <= landing_zone_.x_max &&
                current_y >= landing_zone_.y_min && current_y <= landing_zone_.y_max);
    }
};

// ==============================================================================
// NODO 2: Offboard Mission Originale
// Si occupa di navigazione. Ascolta eventuali segnali di aborto.
// ==============================================================================
class OffboardSquareMission : public rclcpp::Node
{
public:
    OffboardSquareMission() : Node("offboard_square_mission")
    {
        offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Sottoscrizione al topic di aborto inviato dal Geofence Monitor
        abort_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/abort", 10,
            std::bind(&OffboardSquareMission::abort_cb, this, std::placeholders::_1));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&OffboardSquareMission::local_position_cb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&OffboardSquareMission::timer_cb, this));

        init_waypoints();

        RCLCPP_INFO(get_logger(), "Offboard square mission node avviato");
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

    struct Waypoint { float x; float y; float z; };

    /* ROS entities */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_sub_;

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

    // Gestione del messaggio di aborto
    void abort_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && state_ != MissionState::LAND && state_ != MissionState::DONE) {
            RCLCPP_WARN(get_logger(), "Ricevuto segnale di aborto missione! Forzo l'atterraggio.");
            state_ = MissionState::LAND;
        }
    }

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
                RCLCPP_INFO(get_logger(), "Decollo completato");
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
                RCLCPP_INFO(get_logger(), "Waypoint %zu raggiunto", waypoint_index_);
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

    bool reached(const Waypoint &wp) { return reached(wp.x, wp.y, wp.z); }

    bool reached(float tx, float ty, float tz)
    {
        float dx = tx - x_; float dy = ty - y_; float dz = tz - z_;
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
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(get_logger(), "Comando di atterraggio inviato");
    }

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
// MAIN: Esecuzione di entrambi i nodi
// ==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Creiamo le istanze dei due nodi
    auto mission_node = std::make_shared<OffboardSquareMission>();
    auto geofence_node = std::make_shared<GeofenceMonitor>();
    
    // Utilizziamo un Executor per far girare entrambi i nodi all'interno dello stesso processo
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.add_node(geofence_node);
    
    // Lo spin dell'executor si occupa di chiamare le callback di ENTRAMBI i nodi
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}