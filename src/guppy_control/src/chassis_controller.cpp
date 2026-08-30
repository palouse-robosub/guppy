#include "guppy_control/chassis_controller.hpp"

namespace chassis_controller {

bool ChassisController::control_loop(bool debug) {
    // helper to square the magnitude while maintaining direction
    Eigen::Vector<double, 6> desired_squared =
        desired_velocity_state_.cwiseAbs().array()
        * desired_velocity_state_.array();

    // calculate drag effect on sub
    Eigen::Vector<double, 6> drag_plain =
        params_.drag_coefficients.array() * params_.water_density
        * desired_squared.array() * params_.drag_areas.array();

    // apply the drag effect matrix
    auto drag_wrench = params_.drag_effect_matrix * drag_plain;

    // calculate gravity effect on sub
    Eigen::Vector3d gravity_force;
    gravity_force << 0, 0, -(GRAVITY * params_.robot_mass);
    gravity_force = current_orientation_state_.inverse() * gravity_force;
    Eigen::Vector<double, 6> gravity_wrench;
    gravity_wrench << gravity_force[0], gravity_force[1], gravity_force[2], 0,
        0, 0;

    // calculate buoyant effect on sub
    Eigen::Vector3d buoyancy_force;
    buoyancy_force << 0, 0,
        params_.water_density * params_.robot_volume * GRAVITY;
    Eigen::Vector3d r_vec =
        current_orientation_state_.inverse() * params_.center_of_buoyancy;
    Eigen::Vector3d buoyancy_torque = 1 * r_vec.cross(buoyancy_force);
    // if (pose_pid_enabled) {
    buoyancy_torque = -1 * r_vec.cross(buoyancy_force);
    // }
    Eigen::Vector3d buoyancy_force_rotated =
        current_orientation_state_.inverse() * buoyancy_force;
    Eigen::Vector<double, 6> buoyancy_wrench;
    buoyancy_wrench << buoyancy_force_rotated[0], buoyancy_force_rotated[1],
        buoyancy_force_rotated[2], buoyancy_torque;

    // calculate total feedforward
    Eigen::Vector<double, 6> feedforward =
        -(drag_wrench + buoyancy_wrench + gravity_wrench);

    // calculate PID of current velocity error
    Eigen::Vector<double, 6> velocity_feedback;
    for (int i = 0; i < 6; i++) {
        if (abs(desired_velocity_state_[i]) >= params_.pose_lock_deadband[i]) {
            velocity_feedback[i] = velocity_pid[i].compute_command(
                desired_velocity_state_[i] - current_velocity_state_[i],
                dt_us_ / 1000000.0
            );
        } else {
            velocity_feedback[i] = 0;
        }
    }

    // calculate position and orientation pid

    Eigen::Vector<double, 6> added_pose_nudge;
    Eigen::Vector3d          position_nudge = Eigen::Vector3d::Zero();

    // positions...
    Eigen::Vector3d position_err =
        desired_position_state_ - current_position_state_;
    position_err = current_orientation_state_.inverse() * position_err;
    for (int i = 0; i < 3; i++) {
        if (abs(desired_velocity_state_[i]) < params_.pose_lock_deadband[i]
            && pose_pid_enabled) {
            position_nudge[i] = pose_pid[i].compute_command(
                position_err[i], dt_us_ / 1000000.0
            );
        } else {
            position_nudge[i]          = 0;
            desired_position_state_[i] = current_position_state_[i];
        }
    }

    // position_nudge = current_orientation_state_.inverse() * position_nudge;
    // position_nudge = Eigen::Vector3d(-position_nudge[0], -position_nudge[1],
    // position_nudge[2]);

    // orientation...
    Eigen::Vector3d rotational_nudge = calculate_rotational_nudge(debug);
    added_pose_nudge << position_nudge, rotational_nudge;

    // allocate thrust
    auto local_wrench = feedforward + velocity_feedback + added_pose_nudge;
    motor_forces_     = allocate_thrust(local_wrench);

    // convert the Newtons of thrust to -1/1 throttle values
    Eigen::Vector<double, N_MOTORS> motor_throttles;
    for (int i = 0; i < N_MOTORS; i++) {
        double max_in_dir  = motor_forces_[i] < 0
                               ? params_.motor_lower_bounds[i]
                               : params_.motor_upper_bounds[i];
        motor_throttles[i] = motor_forces_[i] / abs(max_in_dir);
    }

    if (debug) {
        std::cout << "buoyancy_wrench:   " << buoyancy_wrench.transpose()
                  << std::endl;
        std::cout << "gravity_wrench:    " << gravity_wrench.transpose()
                  << std::endl;
        std::cout << "drag_wrench:       " << drag_wrench.transpose()
                  << std::endl;
        std::cout << "velocity_feedback: " << velocity_feedback.transpose()
                  << std::endl;
        std::cout << "c pos:             "
                  << current_position_state_.transpose() << std::endl;
        std::cout << "d pos:             "
                  << desired_position_state_.transpose() << std::endl;
        std::cout << "pose_nudge:        " << added_pose_nudge.transpose()
                  << std::endl;
        // std::cout << std::endl;
        std::cout << "local_wrench:      " << local_wrench.transpose()
                  << std::endl;
        std::cout << "motor_forces_:     " << motor_forces_.transpose()
                  << std::endl;
        std::cout << std::endl;
    }

    // write to hardware interface
    bool success = interface_->write(motor_throttles);

    return success;
}

void ChassisController::reset_holding_pose(
    guppy_msgs::srv::SetHoldPose::Request::SharedPtr request
) {
    if (request->type
        == request->guppy_msgs::srv::SetHoldPose::Request::RESET) {
        this->desired_orientation_state_ = this->current_orientation_state_;
        this->desired_position_state_    = this->current_position_state_;
    } else {
        auto ros_quat = request->pose.orientation;
        auto ros_pos  = request->pose.position;

        Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
        Eigen::Vector3d    pos;
        pos << ros_pos.x, ros_pos.y, ros_pos.z;

        if (request->type
            == request->guppy_msgs::srv::SetHoldPose::Request::GLOBAL) {
            this->desired_position_state_    = pos;
            this->desired_orientation_state_ = quat;
        } else if (request->type
                   == request
                          ->guppy_msgs::srv::SetHoldPose::Request::RELATIVE) {
            this->desired_orientation_state_ =
                this->current_orientation_state_ * quat;
            this->desired_position_state_ = this->current_position_state_ + pos;
        } else if (request->type
                   == request->guppy_msgs::srv::SetHoldPose::Request::LOCAL) {
            this->desired_orientation_state_ =
                this->current_orientation_state_ * quat;
            this->desired_position_state_ =
                this->current_position_state_
                + (this->current_orientation_state_.inverse() * pos);
        }
    }
}

Eigen::Vector3d ChassisController::calculate_rotational_nudge(bool debug) {
    if (!pose_pid_enabled) {
        Eigen::Vector3d out(0.0, 0.0, 0.0);
        return out;
    }
    // the new state flags of the rotational locks
    int new_orientation_lock = ALL_FREE;    // == 0

    // update state flags
    if (abs(desired_velocity_state_[3]) < params_.pose_lock_deadband[3])
        new_orientation_lock |= ROLL_LOCK;
    if (abs(desired_velocity_state_[4]) < params_.pose_lock_deadband[4])
        new_orientation_lock |= PITCH_LOCK;
    if (abs(desired_velocity_state_[5]) < params_.pose_lock_deadband[5])
        new_orientation_lock |= YAW_LOCK;

    // make sure to update the desired orientation if needed
    if (((int)current_orientation_lock_ != new_orientation_lock)
        || first_run < 10) {
        if (first_run < 10)
            first_run++;
        desired_orientation_state_ = current_orientation_state_;
        current_orientation_lock_  = (OrientationLockState)new_orientation_lock;
    }

    // calculate the error quaternion
    Eigen::Quaternion q_err =
        current_orientation_state_.inverse() * desired_orientation_state_;
    Eigen::Vector3d axis_err = -1 * q_err.vec();

    // // flip to achieve shortest rotation
    // if (q_err.w() < 0) axis_err = -axis_err;

    // calculate the output nudge with PID
    Eigen::Vector3d output_nudge = Eigen::Vector3d::Zero();
    if (ROLL_LOCK & current_orientation_lock_)
        output_nudge[0] =
            pose_pid[3].compute_command(axis_err[0], dt_us_ / 1000000.0);
    if (PITCH_LOCK & current_orientation_lock_)
        output_nudge[1] =
            pose_pid[4].compute_command(axis_err[1], dt_us_ / 1000000.0);
    if (YAW_LOCK & current_orientation_lock_)
        output_nudge[2] =
            pose_pid[5].compute_command(axis_err[2], dt_us_ / 1000000.0);

    if (debug) {
        std::cout << "lock_state: " << (int)new_orientation_lock << std::endl;
        std::cout << "old_state: " << (int)current_orientation_lock_
                  << std::endl;

        std::cout << "c: " << current_orientation_state_.w() << " "
                  << current_orientation_state_.vec().transpose() << std::endl;
        std::cout << "d: " << desired_orientation_state_.w() << " "
                  << desired_orientation_state_.vec().transpose() << std::endl;
        std::cout << "axis_err: " << axis_err.transpose() << std::endl;
    }

    return output_nudge;
}

void ChassisController::enable_pose_pid(bool enabled) {
    this->pose_pid_enabled = enabled;
}

ChassisController::ChassisController(
    ChassisControllerParams parameters, T200Interface* hw_interface, int dt_us
) :
  interface_(hw_interface), current_orientation_state_(1, 0, 0, 0),
  desired_orientation_state_(1, 0, 0, 0), params_(parameters),
  qp_(N_MOTORS, 0, N_MOTORS), dt_us_(dt_us), THREAD_RUNNING_(false) {
    for (int i = 0; i < 6; i++) {
        this->velocity_pid.push_back(
            control_toolbox::Pid(
                1, 0, 0, std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                ChassisController::antiwindup_strat
            )
        );
        this->pose_pid.push_back(
            control_toolbox::Pid(
                1, 0, 0, std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                ChassisController::antiwindup_strat
            )
        );
    }

    update_parameters(parameters);
}

ChassisController::~ChassisController() {
    if (THREAD_RUNNING_.load()) {
        // join thread and end it
        THREAD_RUNNING_.store(false);
        if (control_thread_.joinable())
            control_thread_.join();
    }
}

Eigen::Vector<double, N_MOTORS>
    ChassisController::allocate_thrust(Eigen::Vector<double, 6> local_wrench) {
    // turn the least squares problem solution set into a QP program
    Eigen::VectorXd qp_g =
        -(params_.motor_coefficients.transpose() * params_.axis_weight_matrix
          * local_wrench);

    // update solver and solve
    qp_.update(
        std::nullopt, qp_g, std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, std::nullopt
    );
    qp_.solve();

    // warm start drastically decreases execution time
    qp_.settings.initial_guess =
        proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    return qp_.results.x;
}

void ChassisController::update_current_state(
    nav_msgs::msg::Odometry::SharedPtr msg
) {
    // get message parts from the Shared Pointer
    auto ros_odom  = msg;
    auto ros_quat  = ros_odom->pose.pose.orientation;
    auto ros_pos   = ros_odom->pose.pose.position;
    auto ros_twist = ros_odom->twist.twist;

    // update current velocity
    Eigen::Vector<double, 6> new_current_vel;
    new_current_vel << ros_twist.linear.x, ros_twist.linear.y,
        ros_twist.linear.z, ros_twist.angular.x, ros_twist.angular.y,
        ros_twist.angular.z;
    this->current_velocity_state_ = new_current_vel;

    // update current orientation
    Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
    this->current_orientation_state_ = quat;

    // update current linear position
    Eigen::Vector3d new_current_pos;
    new_current_pos << ros_pos.x, ros_pos.y, ros_pos.z;
    this->current_position_state_ = new_current_pos;
}

void ChassisController::update_desired_state(
    geometry_msgs::msg::Twist::SharedPtr msg
) {
    auto ros_twist = msg.get();

    // update desired state from ros2 message
    Eigen::Vector<double, 6> new_desired_state;
    new_desired_state << ros_twist->linear.x, ros_twist->linear.y,
        ros_twist->linear.z, ros_twist->angular.x, ros_twist->angular.y,
        ros_twist->angular.z;

    if (std::isnan(ros_twist->linear.x) || std::isnan(ros_twist->linear.y)
        || std::isnan(ros_twist->linear.z)) {
        return;
    }

    this->desired_velocity_state_ = new_desired_state;
}

void ChassisController::update_parameters(ChassisControllerParams parameters) {
    // set parameter object
    this->params_ = parameters;

    // recalculate the motor coefficients into a QP problem
    Eigen::MatrixXd qp_A = params_.motor_coefficients;
    Eigen::MatrixXd qp_H = qp_A.transpose() * params_.axis_weight_matrix * qp_A;
    qp_H += Eigen::Matrix<double, N_MOTORS, N_MOTORS>::Identity() * 0.1;

    // no equality constraints
    Eigen::MatrixXd qp_C = Eigen::MatrixXd::Identity(N_MOTORS, N_MOTORS);

    // update QP settings from params
    qp_.settings.eps_abs         = params_.qp_epsilon;    // convergence amount
    qp_.settings.initial_guess   = InitialGuessStatus::NO_INITIAL_GUESS;
    qp_.settings.verbose         = false;
    qp_.settings.compute_timings = true;

    // update QP vars again
    qp_.init(
        qp_H, std::nullopt, std::nullopt, std::nullopt, qp_C,
        params_.motor_lower_bounds, params_.motor_upper_bounds
    );

    // set all PID gains
    // this is slightly annoying, but the library requires a format like this
    this->velocity_pid[0].set_gains(
        params_.pid_gains_vel_linear[0], params_.pid_gains_vel_linear[1],
        params_.pid_gains_vel_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->velocity_pid[1].set_gains(
        params_.pid_gains_vel_linear[0], params_.pid_gains_vel_linear[1],
        params_.pid_gains_vel_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->velocity_pid[2].set_gains(
        params_.pid_gains_vel_linear[0], params_.pid_gains_vel_linear[1],
        params_.pid_gains_vel_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->velocity_pid[3].set_gains(
        params_.pid_gains_vel_angular[0], params_.pid_gains_vel_angular[1],
        params_.pid_gains_vel_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->velocity_pid[4].set_gains(
        params_.pid_gains_vel_angular[0], params_.pid_gains_vel_angular[1],
        params_.pid_gains_vel_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->velocity_pid[5].set_gains(
        params_.pid_gains_vel_angular[0], params_.pid_gains_vel_angular[1],
        params_.pid_gains_vel_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );

    this->pose_pid[0].set_gains(
        params_.pid_gains_pose_linear[0], params_.pid_gains_pose_linear[1],
        params_.pid_gains_pose_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->pose_pid[1].set_gains(
        params_.pid_gains_pose_linear[0], params_.pid_gains_pose_linear[1],
        params_.pid_gains_pose_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->pose_pid[2].set_gains(
        params_.pid_gains_pose_linear[0], params_.pid_gains_pose_linear[1],
        params_.pid_gains_pose_linear[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->pose_pid[3].set_gains(
        params_.pid_gains_pose_angular[0], params_.pid_gains_pose_angular[1],
        params_.pid_gains_pose_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->pose_pid[4].set_gains(
        params_.pid_gains_pose_angular[0], params_.pid_gains_pose_angular[1],
        params_.pid_gains_pose_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
    this->pose_pid[5].set_gains(
        params_.pid_gains_pose_angular[0], params_.pid_gains_pose_angular[1],
        params_.pid_gains_pose_angular[2],
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup_strat
    );
}

ChassisController::ChassisControllerParams
    ChassisController::get_param_struct() {
    return this->params_;
}

Eigen::Vector<double, N_MOTORS> ChassisController::get_motor_thrusts() {
    // simple getter
    return this->motor_forces_;
}

void ChassisController::loop_runner(bool debug) {
    // init interface (send 0 to all...)
    interface_->initialize();

    // keep track of min and max loop times
    int min_us = 99999;    // arbitraily large value...
    int max_us = 0;

    // loop until atomic shutdown flag
    while (THREAD_RUNNING_.load()) {
        // calculate next wake given dt_us
        auto next_wake = steady_clock::now() + microseconds(dt_us_);
        auto start     = high_resolution_clock::now();

        // actually run the control code here
        bool okay = control_loop(debug);

        // time the loop
        auto stop        = high_resolution_clock::now();
        int  duration_us = duration_cast<microseconds>(stop - start).count();

        // update min/max
        if (duration_us < min_us)
            min_us = duration_us;
        if (duration_us > max_us)
            max_us = duration_us;

        if (debug) {
            std::cout << duration_us << " us total loop time" << std::endl;
            if (!okay)
                std::cerr << "ERROR WRITING TO HARDWARE INTERFACE" << std::endl;
        }

        // wait until next loop to maintain dt_us consistency
        std::this_thread::sleep_until(next_wake);
    }

    // shutdown interface (send 0 to all...)
    interface_->shutdown();

    if (debug)
        std::cout << "\t\t min_us:" << min_us << "\tmax_us: " << max_us
                  << std::endl;
}

void ChassisController::start(bool debug) {
    // startup the thread
    THREAD_RUNNING_.store(true);
    control_thread_ = std::thread(&ChassisController::loop_runner, this, debug);
}

void ChassisController::stop() {
    // end the thread (prefered over the destructor)
    THREAD_RUNNING_.store(false);
    if (control_thread_.joinable())
        control_thread_.join();
}

}    // namespace chassis_controller
