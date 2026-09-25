#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include <atomic>
#include <thread>
#include <vector>
#include <mutex>

#include <Eigen/Core>
#include <control_toolbox/control_toolbox/pid.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "guppy_msgs/srv/set_hold_pose.hpp"
#include "guppy_control/t200_interface.hpp"

using namespace std::chrono_literals;

/* stores the state of the orientation locking code */
enum class OrientationLockState : uint8_t {
    AllFree       = 0b000, /* All axes are free: none are locked */

    RollLock      = 0b100, /* Roll (X) axis is locked */
    PitchLock     = 0b010, /* Pitch (Y) axis is locked */
    YawLock       = 0b001, /* Yaw (Z) axis is locked */

    RollPitchLock = 0b110, /* Roll and Pitch (X and Y) axes are locked */
    RollYawLock   = 0b101, /* Roll and Yaw (X and Z) axes are locked */
    PitchYawLock  = 0b011, /* Pitch and Yaw (Y and Z) axes are locked */

    AllLocked     = 0b111  /* all rotation axes are locked: station keeping */
};

constexpr OrientationLockState operator&(OrientationLockState left, OrientationLockState right) {
    return static_cast<OrientationLockState>(static_cast<uint8_t>(left) & static_cast<uint8_t>(right));
}

constexpr OrientationLockState operator|(OrientationLockState left, OrientationLockState right) {
    return static_cast<OrientationLockState>(static_cast<uint8_t>(left) | static_cast<uint8_t>(right));
}

constexpr OrientationLockState& operator|=(OrientationLockState& left, OrientationLockState right) {
    left = left | right;
    return left;
}

bool inline has_lock(OrientationLockState state, OrientationLockState lock) {
    return (state & lock) != OrientationLockState::AllFree;
}

class ChassisController {
public:
    struct Parameters {
        /* A 6XT200Interface::motor_count matrix of coefficients corresponding to each motor */
        Eigen::Matrix<double, 6, T200Interface::motor_count> motor_coefficients =
            Eigen::Matrix<double, 6, T200Interface::motor_count>::Zero();
        /* A vector of ordered motor lower bounds in Newtons of thrust. */
        Eigen::Vector<double, T200Interface::motor_count> motor_lower_bounds =
            Eigen::Vector<double, T200Interface::motor_count>::Zero();
        /* A vector of ordered motor upper bounds in Newtons of thrust. */
        Eigen::Vector<double, T200Interface::motor_count> motor_upper_bounds =
            Eigen::Vector<double, T200Interface::motor_count>::Zero();
        // control parameters
        /* A diagonal matrix of weights between Fx,Fy,Fz,Tx,Ty,Tz in the QP
         * problem
         */
        Eigen::Matrix<double, 6, 6> axis_weight_matrix =
            Eigen::Matrix<double, 6, 6>::Identity();
        /* PID Gains for linear velocity feedback control */
        std::vector<double> pid_gains_vel_linear = { 1, 0, 0 };
        /* PID Gains for angular velocity feedback control */
        std::vector<double> pid_gains_vel_angular = { 1, 0, 0 };
        /* PID Gains for linear position feedback control */
        std::vector<double> pid_gains_pose_linear = { 1, 0, 0 };
        /* PID Gains for angular orientation feedback control */
        std::vector<double> pid_gains_pose_angular = { 1, 0, 0 };
        /* Six deadband values for when an axis should be considered locked */
        Eigen::Vector<double, 6> pose_lock_deadband =
            Eigen::Vector<double, 6>::Zero();
        // robot setup
        /* Drag coefficients for movement in all six axes */
        Eigen::Vector<double, 6> drag_coefficients =
            Eigen::Vector<double, 6>::Zero();
        /* Drag areas of all six "axes" */
        Eigen::Vector<double, 6> drag_areas = Eigen::Vector<double, 6>::Zero();
        /*
            A 6x6 matrix which is multiplied by drag force to predict offsets in
           movement caused by drag moment arms. For example:

                drag in:      Fx  Fy  Fz  Tx  Ty  Tz
                           x  1   0   0   0   0   0
                           y  0   1   0   0   0   0
            causes extra:  z  0   0   1   0   0   0
                           r  0   0   0   1   0   0
                           p  0   0   0   0   1   0
                           y  0   .2  0   0   0   1

            The above matrix would mean that pitch is changed by 0.2Fx movement.
        */
        Eigen::Matrix<double, 6, 6> drag_effect_matrix =
            Eigen::Matrix<double, 6, 6>::Identity();
        /* water density in kg/m^3 */
        double water_density = 1000;    // kg/m^3
        /* robot volume in m^3 */
        double robot_volume;    // m^3
        /* robot mass in kg */
        double robot_mass;    // kg
        /* center of buoyancy arm from CM (in meters)*/
        Eigen::Vector3<double> center_of_buoyancy =
            Eigen::Vector3<double>::Zero();
        // qp solver
        /* QP convergence epsilon */
        double qp_epsilon = 1e-2;
    };
private:
    static constexpr const inline auto gravity = 9.81;

    unsigned int first_run = 0;

    static const inline auto strategy_factory =
        []() {
            control_toolbox::AntiWindupStrategy strategy;
            strategy.set_type("back_calculation");
            return strategy;
        };

    static const inline auto anti_windup_strategy = strategy_factory();
private:
    const std::shared_ptr<T200Interface> interface_;

    Eigen::Vector<double, 6> current_velocity_state_;
    Eigen::Quaternion<double> current_orientation_state_{1, 0, 0, 0};
    Eigen::Vector3d current_position_state_;
    Eigen::Quaternion<double> desired_orientation_state_{1, 0, 0, 0};
    Eigen::Vector3d desired_position_state_;
    Eigen::Vector<double, 6> desired_velocity_state_;

    OrientationLockState current_orientation_lock_ = OrientationLockState::AllFree;

    Eigen::Vector<double, T200Interface::motor_count> motor_forces_;

    Parameters parameters_;
    /*
     * look into semaphore!
     */
    std::mutex parameter_mutex_;

    std::vector<control_toolbox::Pid> velocity_pid;
    std::vector<control_toolbox::Pid> pose_pid;

    proxsuite::proxqp::dense::QP<double> qp_{T200Interface::motor_count, 0, T200Interface::motor_count};

    const unsigned long dt_us_;
    const double dt_;

    std::atomic<bool> is_thread_running_ = false;
    std::thread control_thread_;

    bool pose_pid_enabled_ = false;
public:
    ChassisController(Parameters parameters, const std::shared_ptr<T200Interface> hw_interface, unsigned long dt_us);
    ~ChassisController();
    void update_current_state(const nav_msgs::msg::Odometry& msg);
    void update_desired_state(const geometry_msgs::msg::Twist& msg);
    void reset_holding_pose(std::shared_ptr<const guppy_msgs::srv::SetHoldPose::Request> request);
    void enable_pose_pid(bool enabled);
    void update_parameters(const Parameters& parameters);
    ChassisController::Parameters get_param_struct();
    void start(bool debug);
    void stop();
    Eigen::Vector<double, T200Interface::motor_count> get_motor_thrusts();
private:
    Eigen::Vector<double, T200Interface::motor_count> allocate_thrust(const Eigen::Vector<double, 6>& local_wrench);
    Eigen::Vector3d calculate_rotational_nudge(bool debug);
    bool control_loop(bool debug);
    void loop_runner(bool debug);
};

#endif
