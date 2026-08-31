#ifndef GUPPY_CHASSIS_CONTROLLER_H
#define GUPPY_CHASSIS_CONTROLLER_H

#define GRAVITY  9.81
#define N_MOTORS 8

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "guppy_msgs/srv/set_hold_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Core>
#include <control_toolbox/control_toolbox/pid.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include "guppy_control/t200_interface.hpp"

namespace chassis_controller {

using namespace proxsuite::proxqp;
using namespace t200_interface;
using namespace std::chrono_literals;
using namespace std::chrono;

/* A controller that manages PID, feedforward and thrust allocation for the
 * chassis */
class ChassisController {
  public:
    int first_run = 0;

    /* strange chatgpt magic to bypass the deprecated constructor... */
    inline static const control_toolbox::AntiWindupStrategy antiwindup_strat =
        [] {
            control_toolbox::AntiWindupStrategy a;
            a.set_type("back_calculation");
            return a;
        }();

    // end weird magic

    /* stores the state of the orientation locking code */
    typedef enum orientation_lock_state_struct_ {
        ALL_LOCKED = 0b111, /* all rotation axes are locked: station keeping */
        ROLL_PITCH_LOCK = 0b110, /* Roll and Pitch (X and Y) axes are locked */
        ROLL_YAW_LOCK   = 0b101, /* Roll and Yaw (X and Z) axes are locked */
        ROLL_LOCK       = 0b100, /* Roll (X) axis is locked */
        PITCH_YAW_LOCK  = 0b011, /* Pitch and Yaw (Y and Z) axes are locked */
        PITCH_LOCK      = 0b010, /* Pitch (Y) axis is locked */
        YAW_LOCK        = 0b001, /* Yaw (Z) axis is locked */
        ALL_FREE        = 0b000  /* All axes are free: none are locked */
    } OrientationLockState;

    /* A grouping of parameters to setup and configure the chassis controller.
     */
    typedef struct chassis_controller_params_ {
        /* A 6XN_MOTORS matrix of coefficients corresponding to each motor */
        Eigen::Matrix<double, 6, N_MOTORS> motor_coefficients =
            Eigen::Matrix<double, 6, N_MOTORS>::Zero();
        /* A vector of ordered motor lower bounds in Newtons of thrust. */
        Eigen::Vector<double, N_MOTORS> motor_lower_bounds =
            Eigen::Vector<double, N_MOTORS>::Zero();

        /* A vector of ordered motor upper bounds in Newtons of thrust. */
        Eigen::Vector<double, N_MOTORS> motor_upper_bounds =
            Eigen::Vector<double, N_MOTORS>::Zero();

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
    } ChassisControllerParams;

    /*
        @brief Create a new ChassisController with the given parameters and
       hardware interface
        @param parameters the configuration object for the controller. Can be
       updated with update_parameters(...)
        @param hw_interface the interface that motor values will be written to
        @param dt_us the loop period in microseconds (default 500us)
    */
    ChassisController(
        ChassisControllerParams parameters, T200Interface* hw_interface,
        int dt_us = 500
    );
    ~ChassisController();

    /*
        @brief update the current robot position/orientation/velocity state from
       a Odometry message
        @param msg a SharedPointer to an Odometry ROS2 message object
    */
    void update_current_state(nav_msgs::msg::Odometry::SharedPtr msg);

    /*
        @brief update the desired robot velocity target state from a Twist
       message
        @param msg a SharedPointer to an Twist ROS2 message object
    */
    void update_desired_state(geometry_msgs::msg::Twist::SharedPtr msg);

    /*
        @brief resets the holding pose
    */
    void reset_holding_pose(
        guppy_msgs::srv::SetHoldPose::Request::SharedPtr msg
    );

    /*
        @brief enable pose pid
    */
    void enable_pose_pid(bool enabled);

    /*
        @brief update the configuration parameters
        @param parameters the new configuration options object
    */
    void update_parameters(ChassisControllerParams parameters);

    /*
        @brief get a copy of the configuration paramters
    */
    ChassisControllerParams get_param_struct();

    /*
        @brief initializes hardware interface and starts the control thread
    */
    void start(bool debug);

    /*
        @brief shuts down interface and ends the control thread
    */
    void stop();

    /*
        @brief get commanded thrusts (not normalized -1/1 throttles)
        @return the the commanded motor thrusts in Newtons
    */
    Eigen::Vector<double, N_MOTORS> get_motor_thrusts();

  private:
    /* the hardware interface */
    T200Interface* interface_;

    /* the current velocity state of the submarine (updated by Odometry msgs) */
    Eigen::Vector<double, 6> current_velocity_state_;

    /* the current orientation state of the submarine as a quaternion (updated
     * by Odometry messages) */
    Eigen::Quaternion<double> current_orientation_state_;

    /* the current linear position state of the submarine (updated by Odometry
     * messages) */
    Eigen::Vector3d current_position_state_;

    /* the desired orientation state of the submarine (updated internally
     * through pose lock system) */
    Eigen::Quaternion<double> desired_orientation_state_;

    /* the desired position state of the submarine (updated internally through
     * pose lock system) */
    Eigen::Vector3d desired_position_state_;

    /* the desired velocity state (updated by Twist msgs) */
    Eigen::Vector<double, 6> desired_velocity_state_;

    /* the current lock state of the orientation of the sub */
    OrientationLockState current_orientation_lock_ = ALL_FREE;

    /* the output motor forces in Newtons (using in get_motor_thrusts) */
    Eigen::Vector<double, N_MOTORS> motor_forces_;

    /* the storage location for the setup param object */
    ChassisControllerParams params_;

    /* the PID controllers for velocity control */
    std::vector<control_toolbox::Pid> velocity_pid;

    /* the PID controllers for locked position control */
    std::vector<control_toolbox::Pid> pose_pid;

    /* the QP solver object */
    dense::QP<double> qp_;

    /* the timestep in microseconds of the control loop */
    int dt_us_;

    /* atomic shutoff flag for the control thread */
    std::atomic<bool> THREAD_RUNNING_;

    /* the control thread object */
    std::thread control_thread_;

    /* whether or not pose pid is enabled */
    bool pose_pid_enabled = false;

    // private methods

    /*
        @brief allocates local wrench to motors
        @param local_wrench a 6-vector of <Fx, Fy, Fz, Tx, Ty, Tz> forces in
       Newtons in the local frame
        @return a N_MOTOR-vector of ordered motor thrusts in Newtons
    */
    Eigen::Vector<double, N_MOTORS>
        allocate_thrust(Eigen::Vector<double, 6> local_wrench);

    /*
        @brief calculates the orientation lock logic
        @return a Vector3d of additional torque "nudges" to hold position lock
    */
    Eigen::Vector3d calculate_rotational_nudge(bool debug);

    /*
        @brief the main control loop
        @return a boolean of whether or not the write to the hardware interface
       was successful
    */
    bool control_loop(bool debug);

    /*
        @brief a wrapper for the control_loop() function which loops it and
       handles timing
    */
    void loop_runner(bool debug);
};

}    // namespace chassis_controller

#endif
