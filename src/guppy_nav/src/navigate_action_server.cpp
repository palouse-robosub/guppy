#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "guppy_msgs/action/navigate.hpp"
#include "guppy_nav/trajectory.hpp"
#include "guppy_util/quality.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <control_toolbox/control_toolbox/pid.hpp>
#include <guppy_util/quality.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

class NavigateActionServer : public rclcpp::Node {
public:
    static constexpr const auto threshold      = 0.1;
    static constexpr const auto attack         = 0.4;
    static constexpr const auto decay          = 0.4;
    static constexpr const auto target_rate_ms = 10;
    static constexpr const auto pi             = 3.14159265358979;
    static_assert(attack + decay <= 1.0);
    static_assert(attack >= 0.0);
    static_assert(attack <= 1.0);
    static_assert(decay >= 0.0);
    static_assert(decay <= 1.0);

    static const inline auto strategy_factory = []() {
        control_toolbox::AntiWindupStrategy strategy;
        strategy.set_type("back_calculation");
        return strategy;
    };

    static const inline auto anti_windup_strategy = strategy_factory();
private:
    std::shared_ptr<rclcpp::CallbackGroup> action_callback_group_;
    std::shared_ptr<rclcpp::CallbackGroup> sub_callback_group_;

    std::shared_ptr<rclcpp_action::Server<guppy_msgs::action::Navigate>> action_server_;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>  command_velocity_pub_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub_;

    struct KinematicState {
        geometry_msgs::msg::Pose  pose;
        geometry_msgs::msg::Twist twist;
    };

    NavigateActionServer::KinematicState kinematic_state_;

    control_toolbox::Pid x_pid_, y_pid_, z_pid_, yaw_pid_, pitch_pid_, roll_id_;

    struct Trajectory3 {
        Trajectory x, y, z;

        explicit Trajectory3(
            const Eigen::Vector3d& start_velocity, const Eigen::Vector3d& end_velocity,
            double attack, double decay, double total_time, const Eigen::Vector3d& target_position
        ) :
            x(Trajectory(
                start_velocity.x(), end_velocity.x(), attack, decay, total_time, target_position.x()
            )),
            y(Trajectory(
                start_velocity.y(), end_velocity.y(), attack, decay, total_time, target_position.y()
            )),
            z(Trajectory(
                start_velocity.z(), end_velocity.z(), attack, decay, total_time, target_position.z()
            )) { }

        Eigen::Vector3d velocity(double elapsed) const {
            return Eigen::Vector3d(
                x.getTargetVelocity(elapsed), y.getTargetVelocity(elapsed),
                z.getTargetVelocity(elapsed)
            );
        }

        Eigen::Vector3d position(double elapsed) const {
            return Eigen::Vector3d(
                x.getTargetPosition(elapsed), y.getTargetPosition(elapsed),
                z.getTargetPosition(elapsed)
            );
        }
    };

    struct OrientationSolver {
        Eigen::Quaterniond initial_orientation,
            final_orientation;    // relative to initial, i.e. delta orientation
        double total_time;

        explicit OrientationSolver(
            const Eigen::Quaterniond& initial_orientation /*world frame*/,
            Eigen::Quaterniond& final_orientation /*world frame*/, double total_time
        ) :
            initial_orientation(initial_orientation), final_orientation(final_orientation),
            total_time(total_time) {
            if (initial_orientation.dot(final_orientation) < 0.0) {
                Eigen::Quaterniond second(
                    -final_orientation.w(), -final_orientation.x(), -final_orientation.y(),
                    -final_orientation.z()
                );
                final_orientation = second;
            }
        }

        Eigen::Vector3d error(
            double elapsed, const Eigen::Quaterniond& current_orientation /*world frame*/
        ) const {
            double alpha = std::clamp(elapsed / total_time, 0.0, 1.0);

            Eigen::Quaterniond targetOrientation =
                initial_orientation.slerp(alpha, final_orientation);

            Eigen::Quaterniond orientation_error =
                current_orientation.inverse() * target_orientation;

            // Eigen::AngleAxisd angleAxisError(orientationError);
            // double angle = angleAxisError.angle();
            // if (angle > PI) angle -= 2.0 * PI;

            return orientation_error.vec();
        }
    };
public:
    explicit NavigateActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
        Node("navigate_action_server", options) {
        // action_cb_group_ =
        // this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // sub_cb_group_ =
        // this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto handleGoal = [this](
                              const rclcpp_action::GoalUUID&,
                              std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal
                          ) {
            RCLCPP_INFO(
                this->get_logger(), "goal request with pose and relative set to %d",
                goal->local
            );    // goal->pose

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel =
            [this](
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>>
            ) {
                RCLCPP_INFO(this->get_logger(), "request to cancel goal");

                return rclcpp_action::CancelResponse::ACCEPT;
            };

        auto handle_accepted =
            [this](
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>>
                    goal_handle
            ) {
                std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
            };

        action_server_ = rclcpp_action::create_server<guppy_msgs::action::Navigate>(
            this, "/navigate", handle_goal, handle_cancel, handle_accepted,
            rcl_action_server_get_default_options()    //,
                                                       // action_cb_group_
        );

        rclcpp::SubscriptionOptions odom_options;
        odom_options.callback_group = sub_callback_group_;

        command_velocity_pub_ =
            create_publisher<geometry_msgs::msg::Twist>("cmd_vel/nav", quality::volatile_profile);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", quality::volatile_profile,
            [this](const std::shared_ptr<nav_msgs::msg::Odometry>& msg) {
                this->odometryCallback(msg);
            },
            odom_options
        );

        x_pid_.set_gains(
            0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
        y_pid_.set_gains(
            0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
        z_pid_.set_gains(
            -0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
        yaw_pid_.set_gains(
            0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
        pitch_pid_.set_gains(
            0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
        roll_pid_.set_gains(
            0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(), anti_windup_strategy
        );
    }

    void odometry_callback(const std::shared_ptr<nav_msgs::msg::Odometry>& msg) {
        this->kinematic_state_.pose  = msg->pose.pose;
        this->kinematic_state_.twist = msg->twist.twist;
    }
private:
    KinematicState get_kinematic_state() {
        return this->kinematic_state_;
    }

    geometry_msgs::msg::Twist computeCommandVelocity(
        const Trajectory3& trajectory, const OrientationSolver& orientation_solver, double elapsed,
        double delta, const KinematicState& state, const Eigen::Vector3d& initial_position
    ) {
        const auto relative_target_position =
            trajectory.position(elapsed);    // relative to guppy starting position, world frame
        const auto target_velocity = trajectory.velocity(elapsed);    // world frame

        const Eigen::Vector3d current_position(
            state.pose.position.x, state.pose.position.y,
            state.pose.position.z
        );    // world
        const auto relative_position =
            current_position - initial_position;    // world, relative to the initial position, as
                                                    // if guppy's initial position were (0, 0, 0)
        const auto error =
            relative_target_position - relative_position;    // end - start; target - current; gives
        // vector from current to target, i.e. error

        Eigen::Quaterniond current_orientation(
            state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y,
            state.pose.orientation.z
        );    // world frame
        auto       local_target_velocity = current_orientation.inverse() * target_velocity;
        const auto local_error           = current_orientation.inverse() * error;
        const auto angular_error_local   = orientation_solver.error(elapsed, current_orientation);

        geometry_msgs::msg::Twist command_velocity;

        command_velocity.angular.z =
            yaw_pid_.compute_command(angular_error_local.z(), rclcpp::Duration::from_seconds(delta));
        command_velocity.angular.y = pitch_pid_.compute_command(
            angular_error_local.y(), rclcpp::Duration::from_seconds(delta)
        );
        command_velocity.angular.x = roll_pid_.compute_command(
            angular_error_local.x(), rclcpp::Duration::from_seconds(delta)
        );

        local_target_velocity = local_target_velocity * 1;
        command_velocity.linear.x =
            local_target_velocity.x()
            + x_pid_.compute_command(local_error.x(), rclcpp::Duration::from_seconds(delta));
        command_velocity.linear.y =
            local_target_velocity.y()
            + y_pid_.compute_command(local_error.y(), rclcpp::Duration::from_seconds(delta));
        command_velocity.linear.z =
            local_target_velocity.z()
            + z_pid_.compute_command(local_error.z(), rclcpp::Duration::from_seconds(delta));

        return command_velocity;
    };

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goal_handle
    ) {
        const auto& goal = goalHandle->get_goal();

        const auto            initial_state = get_kinematic_state();    // world frame
        const Eigen::Vector3d initial_position(
            initial_state.pose.position.x, initial_state.pose.position.y,
            initial_state.pose.position.z
        );
        const Eigen::Quaterniond initial_orientation(
            initial_state.pose.orientation.w, initial_state.pose.orientation.x,
            initial_state.pose.orientation.y, initial_state.pose.orientation.z
        );

        const Eigen::Vector3d goal_position(
            goal->pose.position.x, goal->pose.position.y,
            goal->pose.position.z
        );    // if local == true, relative to guppy; if local == false it
              // is a world position (not relative to guppy)
        const Eigen::Quaterniond goal_orientation(
            goal->pose.orientation.w, goal->pose.orientation.x, goal->pose.orientation.y,
            goal->pose.orientation.z
        );

        Eigen::Vector3d    final_position;       // world
        Eigen::Quaterniond final_orientation;    // world
        if (goal->local) {
            final_position = initial_position
                           + initial_orientation.inverse()
                                 * goal_position;    // get final position in world based on
                                                     // guppy's position/orientation
            final_orientation = initial_orientation * goal_orientation;
        } else {
            final_position    = goal_position;
            final_orientation = goal_orientation;
        }

        const Eigen::Vector3d initial_velocity(
            initial_state.twist.linear.x, initial_state.twist.linear.y,
            initial_state.twist.linear.z
        );    // world frame
        const auto relative_final_position =
            final_position - initial_position;    // vector between guppy's initial position
                                                  // (world) and target final position (world),
                                                  // position as if guppy were (0, 0, 0)

        Trajectory3 trajectory(
            initial_velocity, Eigen::Vector3d(0.0, 0.0, 0.0), NavigateActionServer::attack,
            NavigateActionServer::decay, goal->timeout,
            relative_final_position
        );    // world frame velocities (will output target
              // velocities in world frame)
        OrientationSolver orientation_solver(
            initial_orientation, final_orientation,
            goal->timeout
        );    // will output target angular velocities

        rclcpp::Rate rate(1000.0 / TARGET_RATE_MS);

        x_pid_.reset();
        y_pid_.reset();
        z_pid_.reset();
        yaw_pid_.reset();
        pitch_pid_.reset();
        roll_pid_.reset();

        auto         clock = this->get_clock();
        rclcpp::Time start = clock->now();
        rclcpp::Time last  = start;

        auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();
        auto result   = std::make_shared<guppy_msgs::action::Navigate::Result>();

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->target_reached = false;
                goal_handle->canceled(result);

                geometry_msgs::msg::Twist zero_twist;    // publish zero twist
                command_velocity_publisher_->publish(zero_twist);

                return;
            }

            auto   now     = clock->now();
            double elapsed = (now - start).seconds();
            double delta   = std::clamp((now - last).seconds(), 1e-4, 1.0);
            last           = now;

            auto state            = getKinematicState();    // world
            auto command_velocity = computeCommandVelocity(
                trajectory, orientation_solver, elapsed, delta, state /*world*/,
                initial_position /*world*/
            );

            Eigen::Vector3d current_position;
            current_position << state.pose.position.x, state.pose.position.y, state.pose.position.z;
            auto relative_current_position = current_position - initial_position;

            feedback->progress.position.x = currentPosition.x(),
            feedback->progress.position.y = currentPosition.y(),
            feedback->progress.position.z = currentPosition.z();

            this->command_velocity_publisher_->publish(command_velocity);
            goal_handle->publish_feedback(feedback);

            if (elapsed >= goal->timeout)
                break;

            rate.sleep();
        }

        geometry_msgs::msg::Twist zero_twist;    // publish zero twist
        this->command_velocity_publisher_->publish(zero_twist);

        if (rclcpp::ok()) {
            result->pose           = this->get_kinematic_state().pose;
            result->target_reached = true;    // shouldn't assume true, fix that
                                              // by calcing the hypotenuse

            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NavigateActionServer)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto                               node = std::make_shared<NavigateActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
