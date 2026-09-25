#include "guppy_control/chassis_controller.hpp"

ChassisController::ChassisController(
    Parameters parameters, const std::shared_ptr<T200Interface> hw_interface, unsigned long dt_us
) :
    interface_(hw_interface), dt_us_(dt_us),
    dt_(std::chrono::duration<double>(std::chrono::microseconds{dt_us}).count()) {
    for (unsigned int i = 0; i < 6; ++i) {
        this->velocity_pid.push_back(
            control_toolbox::Pid(
                1, 0, 0, std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                ChassisController::anti_windup_strategy
            )
        );
        this->pose_pid.push_back(
            control_toolbox::Pid(
                1, 0, 0, std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                ChassisController::anti_windup_strategy
            )
        );
    }

    update_parameters(parameters);
}

ChassisController::~ChassisController() {
    if (this->is_thread_running_.load()) {
        // join thread and end it
        this->is_thread_running_.store(false);
        if (this->control_thread_.joinable())
            this->control_thread_.join();
    }
}

void ChassisController::update_current_state(
    const nav_msgs::msg::Odometry& msg
) {
    // get message parts from the Shared Pointer
    const auto& ros_quat  = msg.pose.pose.orientation;
    const auto& ros_pos   = msg.pose.pose.position;
    const auto& ros_twist = msg.twist.twist;

    // update current velocity
    const Eigen::Vector<double, 6> new_current_vel(
        ros_twist.linear.x, ros_twist.linear.y,
        ros_twist.linear.z, ros_twist.angular.x, ros_twist.angular.y,
        ros_twist.angular.z
    );
    this->current_velocity_state_ = new_current_vel;

    // update current orientation
    const Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
    this->current_orientation_state_ = quat;

    // update current linear position
    const Eigen::Vector3d new_current_pos(ros_pos.x, ros_pos.y, ros_pos.z);
    this->current_position_state_ = new_current_pos;
}

void ChassisController::update_desired_state(
    const geometry_msgs::msg::Twist& msg
) {
    const auto& ros_twist = msg;

    // update desired state from ros2 message
    const Eigen::Vector<double, 6> new_desired_state(
        ros_twist.linear.x, ros_twist.linear.y,
        ros_twist.linear.z, ros_twist.angular.x, ros_twist.angular.y,
        ros_twist.angular.z
    );

    if (std::isnan(ros_twist.linear.x) || std::isnan(ros_twist.linear.y) || std::isnan(ros_twist.linear.z)) {
        return;
    }

    this->desired_velocity_state_ = new_desired_state;
}

void ChassisController::reset_holding_pose(
    std::shared_ptr<const guppy_msgs::srv::SetHoldPose::Request> request
) {
    if (request->type == request->guppy_msgs::srv::SetHoldPose::Request::RESET) {
        this->desired_orientation_state_ = this->current_orientation_state_;
        this->desired_position_state_    = this->current_position_state_;
    } else {
        const auto ros_quat = request->pose.orientation;
        const auto ros_pos  = request->pose.position;

        const Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
        const Eigen::Vector3d    pos(ros_pos.x, ros_pos.y, ros_pos.z);

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

void ChassisController::enable_pose_pid(bool enabled) {
    this->pose_pid_enabled_ = enabled;
}

void ChassisController::update_parameters(const Parameters& parameters) {
    // set parameter object
    this->parameter_mutex_.lock();
    this->parameters_ = std::move(parameters);

    // recalculate the motor coefficients into a QP problem
    const Eigen::MatrixXd qp_A = this->parameters_.motor_coefficients;
    Eigen::MatrixXd qp_H = qp_A.transpose() * this->parameters_.axis_weight_matrix * qp_A;
    qp_H += Eigen::Matrix<double, T200Interface::motor_count, T200Interface::motor_count>::Identity() * 0.1;

    // no equality constraints
    const Eigen::MatrixXd qp_C = Eigen::MatrixXd::Identity(T200Interface::motor_count, T200Interface::motor_count);

    // update QP settings from params
    this->qp_.settings.eps_abs         = this->parameters_.qp_epsilon;    // convergence amount
    this->qp_.settings.initial_guess   = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
    this->qp_.settings.verbose         = false;
    this->qp_.settings.compute_timings = true;

    // update QP vars again
    this->qp_.init(
        qp_H, std::nullopt, std::nullopt, std::nullopt, qp_C,
        this->parameters_.motor_lower_bounds, this->parameters_.motor_upper_bounds
    );

    // set all PID gains
    for (size_t i = 0; i < 3; ++i)
        this->velocity_pid[i].set_gains(
            this->parameters_.pid_gains_vel_linear[0], this->parameters_.pid_gains_vel_linear[1],
            this->parameters_.pid_gains_vel_linear[2],
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), ChassisController::anti_windup_strategy
        );

    for (size_t i = 3; i < 6; ++i)
        this->velocity_pid[i].set_gains(
            this->parameters_.pid_gains_vel_angular[0], this->parameters_.pid_gains_vel_angular[1],
            this->parameters_.pid_gains_vel_angular[2],
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), ChassisController::anti_windup_strategy
        );

    for (size_t i = 0; i < 3; ++i)
        this->pose_pid[i].set_gains(
            this->parameters_.pid_gains_pose_linear[0], this->parameters_.pid_gains_pose_linear[1],
            this->parameters_.pid_gains_pose_linear[2],
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), ChassisController::anti_windup_strategy
        );

    for (size_t i = 3; i < 6; ++i)
        this->pose_pid[i].set_gains(
            this->parameters_.pid_gains_pose_angular[0], this->parameters_.pid_gains_pose_angular[1],
            this->parameters_.pid_gains_pose_angular[2],
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), ChassisController::anti_windup_strategy
        );

    this->parameter_mutex_.unlock();
}

ChassisController::Parameters ChassisController::get_param_struct() {
    return this->parameters_;
}

void ChassisController::start(bool debug) {
    // startup the thread
    this->is_thread_running_.store(true);
    this->control_thread_ = std::thread(&ChassisController::loop_runner, this, debug);
}

void ChassisController::stop() {
    // end the thread (prefered over the destructor)
    this->is_thread_running_.store(false);
    if (this->control_thread_.joinable())
        this->control_thread_.join();
}

Eigen::Vector<double, T200Interface::motor_count> ChassisController::get_motor_thrusts() {
    return this->motor_forces_;
}

// private methods

Eigen::Vector<double, T200Interface::motor_count> ChassisController::allocate_thrust(const Eigen::Vector<double, 6>& local_wrench) {
    // turn the least squares problem solution set into a QP program
    const Eigen::VectorXd qp_g = -(this->parameters_.motor_coefficients.transpose() * this->parameters_.axis_weight_matrix * local_wrench);

    // update solver and solve
    this->qp_.update(
        std::nullopt, qp_g, std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, std::nullopt
    );
    this->qp_.solve();

    // warm start drastically decreases execution time
    this->qp_.settings.initial_guess =
        proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    return qp_.results.x;
}

Eigen::Vector3d ChassisController::calculate_rotational_nudge(bool debug) {
    if (!this->pose_pid_enabled_) {
        const Eigen::Vector3d out(0.0, 0.0, 0.0);
        return out;
    }
    // the new state flags of the rotational locks
    auto new_orientation_lock = OrientationLockState::AllFree;    // == 0

    // update state flags
    if (abs(this->desired_velocity_state_[3]) < this->parameters_.pose_lock_deadband[3])
        new_orientation_lock |= OrientationLockState::RollLock;
    if (abs(this->desired_velocity_state_[4]) < this->parameters_.pose_lock_deadband[4])
        new_orientation_lock |= OrientationLockState::PitchLock;
    if (abs(this->desired_velocity_state_[5]) < this->parameters_.pose_lock_deadband[5])
        new_orientation_lock |= OrientationLockState::YawLock;

    // make sure to update the desired orientation if needed
    if ((this->current_orientation_lock_ != new_orientation_lock) || first_run < 10) {
        if (first_run < 10)
            first_run++;
        this->desired_orientation_state_ = this->current_orientation_state_;
        this->current_orientation_lock_  = new_orientation_lock;
    }

    // calculate the error quaternion
    const Eigen::Quaternion q_err =
        this->current_orientation_state_.inverse() * this->desired_orientation_state_;
    const Eigen::Vector3d axis_err = -1 * q_err.vec();

    // // flip to achieve shortest rotation
    // if (q_err.w() < 0) axis_err = -axis_err;

    // calculate the output nudge with PID
    Eigen::Vector3d output_nudge = Eigen::Vector3d::Zero();
    if (has_lock(current_orientation_lock_, OrientationLockState::RollLock))
        output_nudge[0] = pose_pid[3].compute_command(axis_err[0], this->dt_);
    if (has_lock(current_orientation_lock_, OrientationLockState::PitchLock))
        output_nudge[1] = pose_pid[4].compute_command(axis_err[1], this->dt_);
    if (has_lock(current_orientation_lock_, OrientationLockState::YawLock))
        output_nudge[2] = pose_pid[5].compute_command(axis_err[2], this->dt_);

    if (debug) {
        std::cout << "lock_state: " << static_cast<unsigned int>(new_orientation_lock) << '\n';
        std::cout << "old_state: " << static_cast<unsigned int>(this->current_orientation_lock_) << '\n';
        std::cout << "c: " << this->current_orientation_state_.w() << " " << this->current_orientation_state_.vec().transpose() << '\n';
        std::cout << "d: " << this->desired_orientation_state_.w() << " " << this->desired_orientation_state_.vec().transpose() << '\n';
        std::cout << "axis_err: " << axis_err.transpose() << std::endl;
    }

    return output_nudge;
}

bool ChassisController::control_loop(const bool debug) {
    // helper to square the magnitude while maintaining direction
    const Eigen::Vector<double, 6> desired_squared =
        this->desired_velocity_state_.cwiseAbs().array()
        * this->desired_velocity_state_.array();

    // calculate drag effect on sub
    const Eigen::Vector<double, 6> drag_plain =
        this->parameters_.drag_coefficients.array() * this->parameters_.water_density
        * desired_squared.array() * this->parameters_.drag_areas.array();

    // apply the drag effect matrix
    const auto drag_wrench = this->parameters_.drag_effect_matrix * drag_plain;

    // calculate gravity effect on sub
    Eigen::Vector3d gravity_force(0, 0, -(ChassisController::gravity * this->parameters_.robot_mass));
    gravity_force = this->current_orientation_state_.inverse() * gravity_force;
    const Eigen::Vector<double, 6> gravity_wrench(gravity_force[0], gravity_force[1], gravity_force[2], 0, 0, 0);

    // calculate buoyant effect on sub
    const Eigen::Vector3d buoyancy_force(0, 0, this->parameters_.water_density * this->parameters_.robot_volume * ChassisController::gravity);
    const Eigen::Vector3d r_vec = this->current_orientation_state_.inverse() * this->parameters_.center_of_buoyancy;
    Eigen::Vector3d buoyancy_torque = 1 * r_vec.cross(buoyancy_force);
    // if (pose_pid_enabled) {
    buoyancy_torque = -1 * r_vec.cross(buoyancy_force);
    // }
    const Eigen::Vector3d buoyancy_force_rotated = this->current_orientation_state_.inverse() * buoyancy_force;
    const Eigen::Vector<double, 6> buoyancy_wrench(
        buoyancy_force_rotated[0], buoyancy_force_rotated[1], buoyancy_force_rotated[2],
        buoyancy_torque[0], buoyancy_torque[1], buoyancy_torque[2]
    );

    // calculate total feedforward
    const Eigen::Vector<double, 6> feedforward = -(drag_wrench + buoyancy_wrench + gravity_wrench);

    // calculate PID of current velocity error
    Eigen::Vector<double, 6> velocity_feedback;
    for (int i = 0; i < 6; i++) {
        if (abs(this->desired_velocity_state_[i]) >= this->parameters_.pose_lock_deadband[i])
            velocity_feedback[i] = velocity_pid[i].compute_command(
                this->desired_velocity_state_[i] - this->current_velocity_state_[i], dt_
            );
        else
            velocity_feedback[i] = 0;
    }

    // calculate position and orientation pid

    Eigen::Vector3d          position_nudge = Eigen::Vector3d::Zero();
    Eigen::Vector<double, 6> added_pose_nudge;

    // positions...
    Eigen::Vector3d position_err = this->desired_position_state_ - this->current_position_state_;
    position_err = this->current_orientation_state_.inverse() * position_err;
    for (int i = 0; i < 3; i++) {
        if (abs(desired_velocity_state_[i]) < this->parameters_.pose_lock_deadband[i] && this->pose_pid_enabled_)
            position_nudge[i] = pose_pid[i].compute_command(position_err[i], this->dt_);
        else {
            position_nudge[i]          = 0;
            this->desired_position_state_[i] = this->current_position_state_[i];
        }
    }

    // position_nudge = current_orientation_state_.inverse() * position_nudge;
    // position_nudge = Eigen::Vector3d(-position_nudge[0], -position_nudge[1],
    // position_nudge[2]);

    // orientation...
    const Eigen::Vector3d rotational_nudge = calculate_rotational_nudge(debug);
    added_pose_nudge << position_nudge, rotational_nudge;

    // allocate thrust
    const auto local_wrench = feedforward + velocity_feedback + added_pose_nudge;
    this->motor_forces_     = allocate_thrust(local_wrench);

    // convert the Newtons of thrust to -1/1 throttle values
    Eigen::Vector<double, T200Interface::motor_count> motor_throttles;
    for (size_t i = 0; i < T200Interface::motor_count; ++i) {
        double max_in_dir = this->motor_forces_[i] < 0
            ? this->parameters_.motor_lower_bounds[i]
            : this->parameters_.motor_upper_bounds[i];
        motor_throttles[i] = this->motor_forces_[i] / abs(max_in_dir);
    }

    if (debug) {
        std::cout << "buoyancy_wrench:   " << buoyancy_wrench.transpose() << '\n';
        std::cout << "gravity_wrench:    " << gravity_wrench.transpose() << '\n';
        std::cout << "drag_wrench:       " << drag_wrench.transpose() << '\n';
        std::cout << "velocity_feedback: " << velocity_feedback.transpose() << '\n';
        std::cout << "c pos:             " << current_position_state_.transpose() << '\n';
        std::cout << "d pos:             " << desired_position_state_.transpose() << '\n';
        std::cout << "pose_nudge:        " << added_pose_nudge.transpose() << '\n';
        std::cout << "local_wrench:      " << local_wrench.transpose() << '\n';
        std::cout << "motor_forces_:     " << motor_forces_.transpose() << '\n';
        std::cout << std::endl;
    }

    std::array<float, T200Interface::motor_count> throttles;
    for (size_t i = 0; i < T200Interface::motor_count; ++i)
        throttles[i] = static_cast<float>(motor_throttles[i]);
    // write to hardware interface
    bool success = this->interface_->write(throttles);

    return success;
}

void ChassisController::loop_runner(bool debug) {
    // keep track of min and max loop times
    long min_us = 99999;    // arbitraily large value...
    long max_us = 0;

    // loop until atomic shutdown flag
    while (this->is_thread_running_.load()) {
        // calculate next wake given dt_us
        const auto next_wake = std::chrono::steady_clock::now() + std::chrono::microseconds(dt_us_);
        const auto start     = std::chrono::high_resolution_clock::now();

        // actually run the control code here
        this->parameter_mutex_.lock();
        const bool okay = control_loop(debug); // maybe pass in parameters here instead of locking during entire control loop runtime?
        this->parameter_mutex_.unlock();

        // time the loop
        const auto stop         = std::chrono::high_resolution_clock::now();
        const long duration_us  = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

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

    if (debug)
        std::cout << "\t\t min_us:" << min_us << "\tmax_us: " << max_us << std::endl;
}
