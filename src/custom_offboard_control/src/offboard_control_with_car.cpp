#include <rclcpp/rclcpp.hpp>
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

// ==============================================================================
// NODO 0: Ground Vehicle Simulator
// ==============================================================================
class GroundVehicleSimulator : public rclcpp::Node
{
public:
    GroundVehicleSimulator() : Node("ground_vehicle_simulator"),
        rng_(std::random_device{}()),
        vel_dist_(-1.0f, 1.0f),
        dur_dist_(1.0f, 4.0f)
    {
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10);

        move_timer_ = create_wall_timer(50ms,
            std::bind(&GroundVehicleSimulator::update_position, this));

        pick_new_velocity();

        RCLCPP_INFO(get_logger(),
            "Ground Vehicle Simulator avviato. Posizione iniziale: (%.1f, %.1f)", x_, y_);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    rclcpp::TimerBase::SharedPtr dir_timer_;

    std::mt19937 rng_;
    std::uniform_real_distribution<float> vel_dist_;
    std::uniform_real_distribution<float> dur_dist_;

    float x_{1.f}, y_{0.f};
    float vx_{0.f}, vy_{0.f};

    void pick_new_velocity()
    {
        vx_ = vel_dist_(rng_);
        vy_ = vel_dist_(rng_);

        float duration_s = dur_dist_(rng_);
        dir_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(duration_s * 1000)),
            [this]() {
                pick_new_velocity();
                dir_timer_->cancel();
            });

        RCLCPP_DEBUG(get_logger(), "Nuova velocità: vx=%.2f vy=%.2f per %.1fs",
                     vx_, vy_, duration_s);
    }

    void update_position()
    {
        constexpr float dt = 0.05f;
        x_ += vx_ * dt;
        y_ += vy_ * dt;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x_;
        msg.pose.position.y = y_;
        msg.pose.position.z = 0.f;
        pose_pub_->publish(msg);
    }
};

// ==============================================================================
// NODO 1: Geofence Monitor (Circolare)
// ==============================================================================
class GeofenceMonitor : public rclcpp::Node
{
public:
    GeofenceMonitor() : Node("geofence_monitor")
    {
        abort_pub_ = create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&GeofenceMonitor::local_position_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Geofence Monitor avviato. Raggio: %.1f m", GEOFENCE_RADIUS);
    }

private:
    static constexpr float GEOFENCE_RADIUS = 15.0f;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_sub_;
    bool abort_triggered_{false};

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        if (abort_triggered_) return;

        float dist = std::hypot(msg->x, msg->y);
        if (dist >= GEOFENCE_RADIUS) {
            RCLCPP_WARN(get_logger(),
                "Allarme Geofence! dist=%.2f m a (x=%.2f, y=%.2f). Aborto!",
                dist, msg->x, msg->y);

            std_msgs::msg::Bool abort_msg;
            abort_msg.data = true;
            abort_pub_->publish(abort_msg);
            abort_triggered_ = true;
        }
    }
};

// ==============================================================================
// NODO 2: Offboard Mission con Tracking del Veicolo a Terra
//
// Logica di tracking:
//   La funzione track_ground_vehicle() calcola ad ogni ciclo la distanza
//   drone↔macchinina e decide lo stato interno di follow:
//
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │  dist >= MIN_FOLLOW_DIST  →  FOLLOWING                             │
//   │    Il setpoint avanza di FOLLOW_STEP metri nella direzione          │
//   │    della macchinina (mai oltre di essa).                            │
//   │    hold_x_/y_ viene aggiornato alla posizione corrente del DRONE    │
//   │    così se la macchina rallenta e torniamo in HOLD sappiamo dove    │
//   │    siamo fermi.                                                     │
//   │                                                                     │
//   │  dist < MIN_FOLLOW_DIST   →  HOLD                                  │
//   │    Il drone rimane fermo sulla propria posizione corrente.          │
//   │    vehicle_x_/y_ viene comunque aggiornato dalla callback.         │
//   │    Al prossimo ciclo, se dist >= MIN_FOLLOW_DIST (la macchinina si  │
//   │    è allontanata), si rientra automaticamente in FOLLOWING.         │
//   └─────────────────────────────────────────────────────────────────────┘
//
//   Non esiste uno stato separato HOLD nella state machine: è la sola
//   funzione track_ground_vehicle() che gestisce il bilanciamento
//   HOLD/FOLLOW ad ogni ciclo di 100 ms, in modo che il passaggio sia
//   immediato in entrambe le direzioni.
// ==============================================================================
class OffboardTrackingMission : public rclcpp::Node
{
public:
    OffboardTrackingMission() : Node("offboard_tracking_mission")
    {
        offboard_pub_ = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_pub_     = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        cmd_pub_      = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        abort_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/abort", 10,
            std::bind(&OffboardTrackingMission::abort_cb, this, std::placeholders::_1));

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        local_pos_sub_ = create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&OffboardTrackingMission::local_position_cb, this, std::placeholders::_1));

        ground_vehicle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10,
            std::bind(&OffboardTrackingMission::ground_vehicle_cb, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms,
            std::bind(&OffboardTrackingMission::timer_cb, this));

        RCLCPP_INFO(get_logger(),
            "Offboard Tracking Mission avviato. MIN_FOLLOW_DIST=%.1f m FOLLOW_STEP=%.1f m",
            MIN_FOLLOW_DIST, FOLLOW_STEP);
    }

private:
    // Distanza minima sotto cui il drone va in HOLD [m]
    static constexpr float MIN_FOLLOW_DIST = 1.0f;
    // Isteresi: la macchinina deve allontanarsi di almeno questa soglia
    // OLTRE MIN_FOLLOW_DIST prima che il drone riprenda a seguirla.
    // Evita oscillazioni rapide HOLD↔FOLLOW quando la distanza è esattamente
    // sul bordo. (0 = nessuna isteresi)
    static constexpr float HYSTERESIS      = 0.2f;
    // Quota di crociera NED (negativo = alto) [m]
    static constexpr float CRUISE_ALT      = -2.0f;
    // Passo di avanzamento massimo per ciclo verso il target [m]
    static constexpr float FOLLOW_STEP     = 0.8f;

    enum class MissionState { WAIT_OFFBOARD, TAKEOFF, TRACK, LAND, DONE };

    // Sotto-stato del tracking, gestito SOLO dentro track_ground_vehicle()
    enum class FollowState { HOLD, FOLLOWING };

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr            local_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_vehicle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             abort_sub_;

    MissionState state_{MissionState::WAIT_OFFBOARD};
    FollowState  follow_state_{FollowState::HOLD};
    uint64_t     setpoint_counter_{0};

    float drone_x_{0.f}, drone_y_{0.f}, drone_z_{0.f};  // frame NED (da PX4)

    // Posizione macchinina convertita in NED per essere confrontabile con
    // drone_x_/y_. Il simulatore pubblica in ENU (frame ROS/map):
    //   ENU: x=Est,  y=Nord
    //   NED: x=Nord, y=Est
    // Conversione: ned_x = enu_y,  ned_y = enu_x
    float vehicle_x_ned_{0.f}, vehicle_y_ned_{0.f};
    bool  vehicle_data_received_{false};

    // Posizione di hold: viene aggiornata continuamente con la pos. del drone
    // quando si è in FOLLOWING, così in HOLD si sa esattamente dove fermarsi.
    float hold_x_{0.f}, hold_y_{0.f};

    // -------------------------------------------------------------------------
    void abort_cb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data &&
            state_ != MissionState::LAND &&
            state_ != MissionState::DONE)
        {
            RCLCPP_WARN(get_logger(), "Aborto! Atterraggio forzato.");
            state_ = MissionState::LAND;
        }
    }

    void local_position_cb(const VehicleLocalPosition::SharedPtr msg)
    {
        drone_x_ = msg->x;
        drone_y_ = msg->y;
        drone_z_ = msg->z;
    }

    // La callback aggiorna SEMPRE la posizione della macchinina.
    // Converte ENU (frame ROS/map) → NED (frame PX4) così tutte le variabili
    // x_/y_ nel nodo sono nello stesso sistema di riferimento NED:
    //   ned_x = enu_y   (Nord  ← y ENU)
    //   ned_y = enu_x   (Est   ← x ENU)
    void ground_vehicle_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        vehicle_x_ned_ = static_cast<float>(msg->pose.position.y);  // ENU y → NED x
        vehicle_y_ned_ = static_cast<float>(msg->pose.position.x);  // ENU x → NED y
        vehicle_data_received_ = true;
    }

    // -------------------------------------------------------------------------
    void timer_cb()
    {
        publish_offboard_mode();

        if (setpoint_counter_ < 10) {
            setpoint_counter_++;
            publish_setpoint(0.f, 0.f, CRUISE_ALT);
            return;
        }

        if (setpoint_counter_ == 10) {
            setpoint_counter_++;
            enable_offboard();
            arm();
            state_ = MissionState::TAKEOFF;
            RCLCPP_INFO(get_logger(), "Arm + Offboard. Decollo a %.1f m.", -CRUISE_ALT);
        }

        run_mission();
    }

    // -------------------------------------------------------------------------
    void run_mission()
    {
        switch (state_) {

        case MissionState::TAKEOFF:
            publish_setpoint(0.f, 0.f, CRUISE_ALT);
            if (reached(0.f, 0.f, CRUISE_ALT)) {
                hold_x_ = drone_x_;
                hold_y_ = drone_y_;
                state_  = MissionState::TRACK;
                RCLCPP_INFO(get_logger(), "Decollo OK. Tracking attivo.");
            }
            break;

        case MissionState::TRACK:
            if (!vehicle_data_received_) {
                publish_setpoint(hold_x_, hold_y_, CRUISE_ALT);
                break;
            }
            // Log diagnostico periodico: stampa entrambi i frame per debug
            RCLCPP_DEBUG(get_logger(),
                "TRACK | drone NED=(%.2f,%.2f) | veicolo NED=(%.2f,%.2f) | dist=%.2f m",
                drone_x_, drone_y_,
                vehicle_x_ned_, vehicle_y_ned_,
                std::hypot(vehicle_x_ned_ - drone_x_, vehicle_y_ned_ - drone_y_));
            track_ground_vehicle();
            break;

        case MissionState::LAND:
            land();
            state_ = MissionState::DONE;
            break;

        case MissionState::DONE:
            publish_setpoint(hold_x_, hold_y_, CRUISE_ALT);
            break;

        default:
            break;
        }
    }

    // -------------------------------------------------------------------------
    // Logica di tracking con transizione HOLD ↔ FOLLOWING
    // Tutte le coordinate sono in NED, coerenti con drone_x_/y_.
    // -------------------------------------------------------------------------
    void track_ground_vehicle()
    {
        float dx   = vehicle_x_ned_ - drone_x_;
        float dy   = vehicle_y_ned_ - drone_y_;
        float dist = std::hypot(dx, dy);

        // ── Transizione di stato con isteresi ─────────────────────────────────
        if (follow_state_ == FollowState::HOLD) {
            if (dist >= MIN_FOLLOW_DIST + HYSTERESIS) {
                follow_state_ = FollowState::FOLLOWING;
                RCLCPP_INFO(get_logger(),
                    "HOLD → FOLLOWING: macchinina a %.2f m (soglia=%.2f m)",
                    dist, MIN_FOLLOW_DIST + HYSTERESIS);
            }
        } else { // FOLLOWING
            if (dist < MIN_FOLLOW_DIST) {
                follow_state_ = FollowState::HOLD;
                hold_x_ = drone_x_;
                hold_y_ = drone_y_;
                RCLCPP_INFO(get_logger(),
                    "FOLLOWING → HOLD: macchinina a %.2f m (< %.2f m). "
                    "Drone fermo in (%.2f, %.2f)",
                    dist, MIN_FOLLOW_DIST, hold_x_, hold_y_);
            }
        }

        // ── Calcolo setpoint ──────────────────────────────────────────────────
        if (follow_state_ == FollowState::HOLD) {
            publish_setpoint(hold_x_, hold_y_, CRUISE_ALT);

            RCLCPP_DEBUG(get_logger(),
                "HOLD: dist=%.2f m. Setpoint fisso NED (%.2f, %.2f)",
                dist, hold_x_, hold_y_);

        } else {
            float nx   = dx / dist;
            float ny   = dy / dist;
            float step = std::min(FOLLOW_STEP, dist);

            float sp_x = drone_x_ + nx * step;
            float sp_y = drone_y_ + ny * step;

            hold_x_ = drone_x_;
            hold_y_ = drone_y_;

            RCLCPP_DEBUG(get_logger(),
                "FOLLOWING: macchinina NED=(%.2f,%.2f) dist=%.2f m -> setpoint NED=(%.2f,%.2f)",
                vehicle_x_ned_, vehicle_y_ned_, dist, sp_x, sp_y);

            publish_setpoint(sp_x, sp_y, CRUISE_ALT);
        }
    }

    // -------------------------------------------------------------------------
    bool reached(float tx, float ty, float tz, float tol = 0.3f)
    {
        float dx = tx - drone_x_;
        float dy = ty - drone_y_;
        float dz = tz - drone_z_;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < tol;
    }

    void publish_offboard_mode()
    {
        OffboardControlMode msg{};
        msg.position  = true;
        msg.timestamp = now_us();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint(float x, float y, float z)
    {
        TrajectorySetpoint msg{};
        msg.position  = {x, y, z};
        msg.yaw       = 0.f;
        msg.timestamp = now_us();
        traj_pub_->publish(msg);
    }

    void enable_offboard()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
    }

    void land()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(get_logger(), "Comando di atterraggio inviato.");
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0.f, float p2 = 0.f)
    {
        VehicleCommand msg{};
        msg.command          = cmd;
        msg.param1           = p1;
        msg.param2           = p2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        msg.timestamp        = now_us();
        cmd_pub_->publish(msg);
    }

    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }
};

// ==============================================================================
// NODO 3: GazeboVisualizer
// ==============================================================================
class GazeboVisualizer : public rclcpp::Node
{
public:
    explicit GazeboVisualizer(const std::string & world_name = "default")
    : Node("gazebo_visualizer"), world_name_(world_name)
    {
        vehicle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ground_vehicle/local_position", 10,
            std::bind(&GazeboVisualizer::vehicle_cb, this, std::placeholders::_1));

        set_pose_service_ = "/world/" + world_name_ + "/set_pose";

        RCLCPP_INFO(get_logger(), "GazeboVisualizer avviato. Servizio: %s",
            set_pose_service_.c_str());
        RCLCPP_INFO(get_logger(),
            "Assicurati di aver spawnato 'ground_vehicle' e 'geofence_circle' nel world '%s'.",
            world_name_.c_str());
    }

private:
    std::string world_name_;
    std::string set_pose_service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_sub_;
    gz::transport::Node gz_node_;

    void vehicle_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        gz::msgs::Pose req;
        req.set_name("ground_vehicle");
        req.mutable_position()->set_x(msg->pose.position.x);
        req.mutable_position()->set_y(msg->pose.position.y);
        req.mutable_position()->set_z(0.08);
        req.mutable_orientation()->set_x(0.0);
        req.mutable_orientation()->set_y(0.0);
        req.mutable_orientation()->set_z(0.0);
        req.mutable_orientation()->set_w(1.0);

        gz::msgs::Boolean rep;
        bool result = false;
        gz_node_.Request(set_pose_service_, req, 50u, rep, result);

        if (!result) {
            RCLCPP_DEBUG(get_logger(),
                "set_pose non riuscito per 'ground_vehicle'.");
        }
    }
};

// ==============================================================================
// MAIN
//
// Spawn dei modelli SDF prima di avviare i nodi (una tantum via shell):
//
//   export WORLD=default
//
//   gz service -s /world/${WORLD}/create \
//     --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
//     --timeout 5000 \
//     --req "sdf_filename: \"$(pwd)/ground_vehicle.sdf\" name: \"ground_vehicle\""
//
//   gz service -s /world/${WORLD}/create \
//     --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
//     --timeout 5000 \
//     --req "sdf_filename: \"$(pwd)/geofence_circle.sdf\" name: \"geofence\""
// ==============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string world_name = (argc > 1) ? argv[1] : "default";

    auto vehicle_node  = std::make_shared<GroundVehicleSimulator>();
    auto geofence_node = std::make_shared<GeofenceMonitor>();
    auto viz_node      = std::make_shared<GazeboVisualizer>(world_name);
    auto mission_node  = std::make_shared<OffboardTrackingMission>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(vehicle_node);
    executor.add_node(geofence_node);
    executor.add_node(viz_node);
    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}