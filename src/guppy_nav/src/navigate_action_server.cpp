#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <control_toolbox/control_toolbox/pid.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "guppy_nav/trajectory.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "guppy_msgs/action/navigate.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define THRESHOLD 0.1

#define ATTACK 0.4  // from 0.0 - 1.0, ATTACK + DECAY must <= 1.0
#define DECAY 0.4

#define TARGET_RATE_MS 10

#define PI 3.1415

class NavigateActionServer : public rclcpp::Node {
 public:
  explicit NavigateActionServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("navigate_action_server", options) {
    // action_cb_group_ =
    // this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // sub_cb_group_ =
    // this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto handleGoal =
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal) {
          RCLCPP_INFO(this->get_logger(),
                      "goal request with pose and relative set to %d",
                      goal->local);  // goal->pose

          (void)uuid;  // silence unused parameter warning

          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

    auto handleCancel =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   guppy_msgs::action::Navigate>> goalHandle) {
          RCLCPP_INFO(this->get_logger(), "request to cancel goal");

          (void)goalHandle;  // silence unused parameter warning

          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto handleAccepted =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   guppy_msgs::action::Navigate>> goalHandle) {
          std::thread{[this, goalHandle]() { execute(goalHandle); }}.detach();
        };

    _actionServer = rclcpp_action::create_server<guppy_msgs::action::Navigate>(
        this, "/navigate", handleGoal, handleCancel, handleAccepted,
        rcl_action_server_get_default_options()  //,
                                                 // action_cb_group_
    );

    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = sub_cb_group_;

    _commandVelocityPublisher =
        create_publisher<geometry_msgs::msg::Twist>("cmd_vel/nav", 10);
    _odomSubscription = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&NavigateActionServer::odometryCallback, this,
                  std::placeholders::_1),
        odom_options);

    _xPid.set_gains(0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    antiWindupStrategy);
    _yPid.set_gains(0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    antiWindupStrategy);
    _zPid.set_gains(-0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    antiWindupStrategy);
    _yawPid.set_gains(0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<double>::infinity(),
                      antiWindupStrategy);
    _pitchPid.set_gains(0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(),
                        antiWindupStrategy);
    _rollPid.set_gains(0.1, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(),
                       antiWindupStrategy);
  }

  void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
    // std::lock_guard<std::mutex> lock(_kinematiStateMutex);
    // RCLCPP_INFO(this->get_logger(), "new thingy");
    _kinematicState.pose = msg->pose.pose;
    _kinematicState.twist = msg->twist.twist;
  }

 private:
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  rclcpp_action::Server<guppy_msgs::action::Navigate>::SharedPtr _actionServer;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      _commandVelocityPublisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscription;

  struct KinematicState {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist twist;
  };

  KinematicState _kinematicState;
  // std::mutex _kinematiStateMutex;

  control_toolbox::Pid _xPid, _yPid, _zPid, _yawPid, _pitchPid, _rollPid;

  struct Trajectory3 {
    Trajectory x, y, z;

    explicit Trajectory3(const Eigen::Vector3d& startVelocity,
                         const Eigen::Vector3d& endVelocity,
                         double attack,
                         double decay,
                         double totalTime,
                         const Eigen::Vector3d& targetPosition)
        : x(Trajectory(startVelocity.x(),
                       endVelocity.x(),
                       attack,
                       decay,
                       totalTime,
                       targetPosition.x())),
          y(Trajectory(startVelocity.y(),
                       endVelocity.y(),
                       attack,
                       decay,
                       totalTime,
                       targetPosition.y())),
          z(Trajectory(startVelocity.z(),
                       endVelocity.z(),
                       attack,
                       decay,
                       totalTime,
                       targetPosition.z())) {}

    Eigen::Vector3d velocity(double elapsed) const {
      return Eigen::Vector3d(x.getTargetVelocity(elapsed),
                             y.getTargetVelocity(elapsed),
                             z.getTargetVelocity(elapsed));
    }

    Eigen::Vector3d position(double elapsed) const {
      return Eigen::Vector3d(x.getTargetPosition(elapsed),
                             y.getTargetPosition(elapsed),
                             z.getTargetPosition(elapsed));
    }
  };

  struct OrientationSolver {
    Eigen::Quaterniond initialOrientation,
        finalOrientation;  // relative to initial, i.e. delta orientation
    double totalTime;

    explicit OrientationSolver(
        const Eigen::Quaterniond& initialOrientation /*world frame*/,
        Eigen::Quaterniond& finalOrientation /*world frame*/,
        double totalTime)
        : initialOrientation(initialOrientation),
          finalOrientation(finalOrientation),
          totalTime(totalTime) {
      if (initialOrientation.dot(finalOrientation) < 0.0) {
        Eigen::Quaterniond second(-finalOrientation.w(), -finalOrientation.x(),
                                  -finalOrientation.y(), -finalOrientation.z());
        finalOrientation = second;
      }
    }

    Eigen::Vector3d error(
        double elapsed,
        const Eigen::Quaterniond& currentOrientation /*world frame*/) const {
      double alpha = std::clamp(elapsed / totalTime, 0.0, 1.0);

      Eigen::Quaterniond targetOrientation =
          initialOrientation.slerp(alpha, finalOrientation);

      Eigen::Quaterniond orientationError =
          currentOrientation.inverse() * targetOrientation;

      // Eigen::AngleAxisd angleAxisError(orientationError);
      // double angle = angleAxisError.angle();
      // if (angle > PI) angle -= 2.0 * PI;

      return orientationError.vec();
    }
  };

  inline static const control_toolbox::AntiWindupStrategy antiWindupStrategy =
      [] {
        control_toolbox::AntiWindupStrategy strategy;
        strategy.set_type("back_calculation");
        return strategy;
      }();

  KinematicState getKinematicState() { return _kinematicState; }

  geometry_msgs::msg::Twist computeCommandVelocity(
      const Trajectory3& trajectory,
      const OrientationSolver& orientationSolver,
      double elapsed,
      double delta,
      const KinematicState& state,
      const Eigen::Vector3d& initialPosition) {
    const auto relativeTargetPosition = trajectory.position(
        elapsed);  // relative to guppy starting position, world frame
    const auto targetVelocity = trajectory.velocity(elapsed);  // world frame

    const Eigen::Vector3d currentPosition(state.pose.position.x,
                                          state.pose.position.y,
                                          state.pose.position.z);  // world
    const auto relativePosition =
        currentPosition -
        initialPosition;  // world, relative to the initial position, as if
                          // guppy's initial position were (0, 0, 0)
    const auto error =
        relativeTargetPosition -
        relativePosition;  // end - start; target - current; gives vector from
                           // current to target, i.e. error

    Eigen::Quaterniond currentOrientation(
        state.pose.orientation.w, state.pose.orientation.x,
        state.pose.orientation.y, state.pose.orientation.z);  // world frame
    auto localTargetVelocity = currentOrientation.inverse() * targetVelocity;
    const auto localError = currentOrientation.inverse() * error;
    const auto angularErrorLocal =
        orientationSolver.error(elapsed, currentOrientation);

    geometry_msgs::msg::Twist commandVelocity;

    commandVelocity.angular.z = _yawPid.compute_command(
        angularErrorLocal.z(), rclcpp::Duration::from_seconds(delta));
    commandVelocity.angular.y = _pitchPid.compute_command(
        angularErrorLocal.y(), rclcpp::Duration::from_seconds(delta));
    commandVelocity.angular.x = _rollPid.compute_command(
        angularErrorLocal.x(), rclcpp::Duration::from_seconds(delta));

    localTargetVelocity = localTargetVelocity * 1;
    commandVelocity.linear.x =
        localTargetVelocity.x() +
        _xPid.compute_command(localError.x(),
                              rclcpp::Duration::from_seconds(delta));
    commandVelocity.linear.y =
        localTargetVelocity.y() +
        _yPid.compute_command(localError.y(),
                              rclcpp::Duration::from_seconds(delta));
    commandVelocity.linear.z =
        localTargetVelocity.z() +
        _zPid.compute_command(localError.z(),
                              rclcpp::Duration::from_seconds(delta));

    return commandVelocity;
  };

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   guppy_msgs::action::Navigate>> goalHandle) {
    const auto& goal = goalHandle->get_goal();

    const auto initialState = getKinematicState();  // world frame
    const Eigen::Vector3d initialPosition(initialState.pose.position.x,
                                          initialState.pose.position.y,
                                          initialState.pose.position.z);
    const Eigen::Quaterniond initialOrientation(
        initialState.pose.orientation.w, initialState.pose.orientation.x,
        initialState.pose.orientation.y, initialState.pose.orientation.z);

    const Eigen::Vector3d goalPosition(
        goal->pose.position.x, goal->pose.position.y,
        goal->pose.position
            .z);  // if local == true, relative to guppy; if local == false it
                  // is a world position (not relative to guppy)
    const Eigen::Quaterniond goalOrientation(
        goal->pose.orientation.w, goal->pose.orientation.x,
        goal->pose.orientation.y, goal->pose.orientation.z);

    Eigen::Vector3d finalPosition;        // world
    Eigen::Quaterniond finalOrientation;  // world
    if (goal->local) {
      finalPosition = initialPosition +
                      initialOrientation.inverse() *
                          goalPosition;  // get final position in world based on
                                         // guppy's position/orientation
      finalOrientation = initialOrientation * goalOrientation;
    } else {
      finalPosition = goalPosition;
      finalOrientation = goalOrientation;
    }

    const Eigen::Vector3d initialVelocity(
        initialState.twist.linear.x, initialState.twist.linear.y,
        initialState.twist.linear.z);  // world frame
    const auto relativeFinalPosition =
        finalPosition -
        initialPosition;  // vector between guppy's initial position (world) and
                          // target final position (world), position as if guppy
                          // were (0, 0, 0)

    Trajectory3 trajectory(
        initialVelocity, Eigen::Vector3d(0.0, 0.0, 0.0), ATTACK, DECAY,
        goal->timeout,
        relativeFinalPosition);  // world frame velocities (will output target
                                 // velocities in world frame)
    OrientationSolver orientationSolver(
        initialOrientation, finalOrientation,
        goal->timeout);  // will output target angular velocities

    rclcpp::Rate rate(1000.0 / TARGET_RATE_MS);

    _xPid.reset();
    _yPid.reset();
    _zPid.reset();
    _yawPid.reset();
    _pitchPid.reset();
    _rollPid.reset();

    auto clock = this->get_clock();
    rclcpp::Time start = clock->now();
    rclcpp::Time last = start;

    auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();
    auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

    while (rclcpp::ok()) {
      if (goalHandle->is_canceling()) {
        result->target_reached = false;
        goalHandle->canceled(result);

        geometry_msgs::msg::Twist zeroTwist;  // publish zero twist
        _commandVelocityPublisher->publish(zeroTwist);

        return;
      }

      auto now = clock->now();
      double elapsed = (now - start).seconds();
      double delta = std::clamp((now - last).seconds(), 1e-4, 1.0);
      last = now;

      auto state = getKinematicState();  // world
      auto commandVelocity =
          computeCommandVelocity(trajectory, orientationSolver, elapsed, delta,
                                 state /*world*/, initialPosition /*world*/);

      Eigen::Vector3d currentPosition;
      currentPosition << state.pose.position.x, state.pose.position.y,
          state.pose.position.z;
      auto relativeCurrentPosition = currentPosition - initialPosition;

      feedback->progress.position.x = currentPosition.x(),
      feedback->progress.position.y = currentPosition.y(),
      feedback->progress.position.z = currentPosition.z();

      _commandVelocityPublisher->publish(commandVelocity);
      goalHandle->publish_feedback(feedback);

      if (elapsed >= goal->timeout)
        break;

      rate.sleep();
    }

    geometry_msgs::msg::Twist zeroTwist;  // publish zero twist
    _commandVelocityPublisher->publish(zeroTwist);

    if (rclcpp::ok()) {
      result->pose = getKinematicState().pose;
      result->target_reached =
          true;  // shouldn't assume true, fix that by calcing the hypotenuse

      goalHandle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "goal succeeded");
    }
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NavigateActionServer)

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavigateActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
