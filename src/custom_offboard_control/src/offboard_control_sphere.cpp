#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp> // <-- Nuovo include per la visualizzazione
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <random>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;

// ==============================================================================
// NODO 1: Geofence Monitor (Circolare)
// Controlla che il drone non si allontani per più di 15m dal punto di origine.
// ==============================================================================
class CircularGeofenceMonitor : public rclcpp::Node
{
public:
    CircularGeofenceMonitor() : Node("circular_geofence_monitor")
    {
        abort_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&CircularGeofenceMonitor::local_position_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Geofence Circolare avviato. Raggio massimo: %.1f m", max_radius_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;

    const float max_radius_ = 15.0f;
    bool abort_triggered_{false};

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        if (!abort_triggered_) {
            // Calcolo la distanza dal centro (origine 0,0) sul piano X-Y
            float dist_from_center = std::sqrt(msg->x * msg->x + msg->y * msg->y);
            
            if (dist_from_center > max_radius_) {
                RCLCPP_WARN(get_logger(), 
                    "Allarme Geofence! Drone fuori dal raggio massimo (Dist: %.2fm). Invio comando di aborto...", 
                    dist_from_center);
                
                std_msgs::msg::Bool abort_msg;
                abort_msg.data = true;
                abort_pub_->publish(abort_msg);
                
                abort_triggered_ = true; 
            }
        }
    }
};

// ==============================================================================
// NODO 2: Target Simulator
// Pubblica un punto virtuale FISSO a terra e un MARKER per visualizzarlo.
// ==============================================================================
class TargetSimulator : public rclcpp::Node
{
public:
    TargetSimulator() : Node("target_simulator")
    {
        target_pub_ = create_publisher<geometry_msgs::msg::Point>("/target/position", 10);
        
        // Publisher per il marker visivo in RViz2
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/target/marker", 10);
        
        // Aggiorna la posizione 10 volte al secondo
        timer_ = create_wall_timer(100ms, std::bind(&TargetSimulator::timer_cb, this));
        
        RCLCPP_INFO(get_logger(), "Target Simulator avviato. Target fisso a terra a (X: %.1f, Y: %.1f).", tx_, ty_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Coordinate fisse del target. (Es. 5 metri in avanti e 5 a destra, a terra z=0)
    const float tx_{5.0f}, ty_{5.0f}, tz_{0.0f};

    void timer_cb()
    {
        // 1. Pubblica le coordinate per il drone
        geometry_msgs::msg::Point msg;
        msg.x = tx_;
        msg.y = ty_;
        msg.z = tz_;
        target_pub_->publish(msg);

        // 2. Pubblica la sfera rossa (Marker) per visualizzarla in RViz2
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Usa "map" o il frame di riferimento del tuo sistema (es. "odom")
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_namespace";
        marker.id = 0;
        
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Posizione
        marker.pose.position.x = tx_;
        marker.pose.position.y = ty_;
        marker.pose.position.z = tz_;
        marker.pose.orientation.w = 1.0;
        
        // Dimensioni (0.5 metri di diametro)
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        
        // Colore (Rosso acceso)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f; // Opacità (1 = solido, 0 = invisibile)
        
        marker_pub_->publish(marker);
    }
};

// ==============================================================================
// NODO 3: Offboard Follow Mission
// Gestisce decollo e l'inseguimento del bersaglio mantenendo una distanza.
// ==============================================================================
class OffboardFollowMission : public rclcpp::Node
{
public:
    OffboardFollowMission() : Node("offboard_follow_mission")
    {
        offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        cmd_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        abort_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/abort", 10,
            std::bind(&OffboardFollowMission::abort_cb, this, std::placeholders::_1));

        target_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/target/position", 10,
            std::bind(&OffboardFollowMission::target_cb, this, std::placeholders::_1));

        // Nuovo: Publisher per visualizzare il drone in RViz2
        drone_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/drone/marker", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            qos,
            std::bind(&OffboardFollowMission::local_position_cb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&OffboardFollowMission::timer_cb, this));

        RCLCPP_INFO(get_logger(), "Offboard Follow Mission avviata");
    }

private:
    enum class MissionState
    {
        WAIT_OFFBOARD,
        TAKEOFF,
        FOLLOW,
        LAND,
        DONE
    };

    /* ROS entities */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr drone_marker_pub_; // <-- Nuovo
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr abort_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;

    /* State */
    MissionState state_{MissionState::WAIT_OFFBOARD};
    uint64_t setpoint_counter_{0};

    /* Posizione attuale del drone */
    float x_{0.f}, y_{0.f}, z_{0.f}, heading_{0.f}; // <-- Aggiunto heading_ per orientare la freccia

    /* Posizione del bersaglio */
    float target_x_{0.f}, target_y_{0.f}, target_z_{-2.f};

    /* Setpoint attuale da mandare al drone */
    float current_sp_x_{0.f}, current_sp_y_{0.f}, current_sp_z_{-2.f};
    float current_yaw_{0.f}; // Nuovo: angolo di imbardata (yaw) desiderato

    const float follow_distance_threshold_ = 3.0f;
    const float takeoff_tolerance_ = 0.3f;

    /* ---------------- Callbacks ---------------- */

    void abort_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && state_ != MissionState::LAND && state_ != MissionState::DONE) {
            RCLCPP_WARN(get_logger(), "Ricevuto segnale di aborto (Geofence)! Forzo l'atterraggio.");
            state_ = MissionState::LAND;
        }
    }

    void target_cb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        target_x_ = msg->x;
        target_y_ = msg->y;
        target_z_ = msg->z;
    }

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        z_ = msg->z;
        heading_ = msg->heading; // <-- Nuovo: salviamo l'angolo di imbardata (yaw)
    }

    void timer_cb()
    {
        publish_offboard_mode();
        publish_drone_marker(); // <-- Nuovo: pubblichiamo la posizione del drone su RViz ad ogni ciclo

        if (setpoint_counter_ < 10) {
            setpoint_counter_++;
            publish_setpoint(current_sp_x_, current_sp_y_, current_sp_z_, current_yaw_);
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
            // Al decollo mira al centro, quota -2m, orientamento 0
            publish_setpoint(0.f, 0.f, -2.f, 0.f);
            
            if (reached(0.f, 0.f, -2.f)) {
                state_ = MissionState::FOLLOW;
                RCLCPP_INFO(get_logger(), "Decollo completato. Inizio inseguimento target...");
            }
            break;

        case MissionState::FOLLOW:
            {
                // Il bersaglio è a terra, ma vogliamo che il drone voli a una quota sicura
                float safe_hover_z = -2.0f;

                // Calcola differenze di posizione sul piano 2D e in quota rispetto all'hovering
                float dx = target_x_ - x_;
                float dy = target_y_ - y_;
                float dz = safe_hover_z - z_; // Usa la quota di volo sicura invece della z del target a terra
                
                // Calcola costantemente l'angolo di imbardata (yaw) per puntare il muso verso il target a terra
                current_yaw_ = std::atan2(dy, dx);

                // Calcola distanza dal target in 3D (rispetto al punto di hovering sopra di esso)
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                // Se la distanza supera i 3 metri, aggiorna il setpoint di posizione verso il target
                if (dist > follow_distance_threshold_) {
                    current_sp_x_ = target_x_;
                    current_sp_y_ = target_y_;
                    current_sp_z_ = safe_hover_z; // Vola sopra il bersaglio, non a terra
                    
                    // Stampa un log limitato nel tempo per non intasare il terminale
                    static int log_counter = 0;
                    if (log_counter++ % 20 == 0) {
                        RCLCPP_INFO(get_logger(), "Distanza target (%.2fm) > 3m. Inseguimento in corso...", dist);
                    }
                }

                // Invia sempre l'ultimo setpoint valido. La posizione si aggiorna solo se lontano,
                // ma lo yaw si aggiorna sempre per guardare il bersaglio in tempo reale.
                publish_setpoint(current_sp_x_, current_sp_y_, current_sp_z_, current_yaw_);
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

    // <-- Nuova funzione per visualizzare il drone in RViz -->
    void publish_drone_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Stesso frame del target
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "drone_namespace";
        marker.id = 1; // ID diverso dal target (che è 0)
        
        // Usiamo una freccia per vedere l'orientamento (il muso) del drone
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Posizione attuale
        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = z_;
        
        // Conversione matematica base da Yaw (heading_) a Quaternione per RViz
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = std::sin(heading_ / 2.0f);
        marker.pose.orientation.w = std::cos(heading_ / 2.0f);
        
        // Dimensioni della freccia
        marker.scale.x = 1.0; // Lunghezza della freccia (1 metro)
        marker.scale.y = 0.2; // Spessore del fusto
        marker.scale.z = 0.2; // Spessore della punta
        
        // Colore (Blu)
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f; 
        
        drone_marker_pub_->publish(marker);
    }

    bool reached(float tx, float ty, float tz)
    {
        float dx = tx - x_; float dy = ty - y_; float dz = tz - z_;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < takeoff_tolerance_;
    }

    void publish_offboard_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = now_us();
        offboard_pub_->publish(msg);
    }

    // Aggiunto il parametro yaw alla funzione
    void publish_setpoint(float x, float y, float z, float yaw = 0.f)
    {
        TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = yaw; // Applica l'orientamento calcolato
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
// MAIN: Esecuzione di tutti i nodi
// ==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto mission_node = std::make_shared<OffboardFollowMission>();
    auto geofence_node = std::make_shared<CircularGeofenceMonitor>();
    auto target_node = std::make_shared<TargetSimulator>();
    
    // Eseguiamo tutti e tre i nodi in parallelo
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mission_node);
    executor.add_node(geofence_node);
    executor.add_node(target_node);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}