#include "guppy_control/chassis_controller.hpp"
#include "guppy_control/t200_interface.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "guppy_msgs/srv/set_hold_pose.hpp"

#include "std_msgs/msg/float64.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/parameter_event_handler.hpp>

#include <array>
#include <unordered_map>

using namespace std::chrono_literals;
using namespace t200_interface;
using namespace chassis_controller;

class ControlNode : public rclcpp::Node {
public:
    static inline const auto reliable_profile = rclcpp::QoS(10).reliable();
    static inline const auto volatile_profile = rclcpp::QoS(10).best_effort().durability_volatile();
private:
    std::shared_ptr<T200Interface> thruster_interface_;
    std::unique_ptr<ChassisController> controller_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, N_MOTORS> sim_motor_pubs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                   odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                 cmd_vel_sub_;
    rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr                    state_sub_;
    rclcpp::TimerBase::SharedPtr                                               timer_;
    rclcpp::Service<guppy_msgs::srv::SetHoldPose>::SharedPtr                   reset_service_;

    std::shared_ptr<rclcpp::ParameterEventHandler>        parameter_sub_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> parameter_event_callback_handle_;
public:
    ControlNode() : Node("control_node") {
        // declare parameters
        this->declare_parameter<std::vector<double>>(
            "motor_positions", std::vector<double>(5 * N_MOTORS, 0.0)
        );    // flattened 6xN
        this->declare_parameter<std::vector<double>>(
            "motor_lower_bounds", std::vector<double>(N_MOTORS, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "motor_upper_bounds", std::vector<double>(N_MOTORS, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "axis_weight_matrix", std::vector<double>(6 * 6, 0.0)
        );    // flattened 6 * 6
        this->declare_parameter<std::vector<double>>(
            "pid_gains_vel_linear", std::vector<double>{ 0.0, 0.0, 0.0 }
        );
        this->declare_parameter<std::vector<double>>(
            "pid_gains_vel_angular", std::vector<double>{ 0.0, 0.0, 0.0 }
        );
        this->declare_parameter<std::vector<double>>(
            "pid_gains_pose_linear", std::vector<double>{ 0.0, 0.0, 0.0 }
        );
        this->declare_parameter<std::vector<double>>(
            "pid_gains_pose_angular", std::vector<double>{ 0.0, 0.0, 0.0 }
        );
        this->declare_parameter<std::vector<double>>(
            "pose_lock_deadband", std::vector<double>(6, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "drag_coefficients", std::vector<double>(6, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "drag_areas", std::vector<double>(6, 0.0)
        );
        this->declare_parameter<std::vector<double>>(
            "drag_effect_matrix", std::vector<double>(6 * 6, 0.0)
        );    // flattened 6x6
        this->declare_parameter<double>("water_density", 0.0);
        this->declare_parameter<double>("robot_volume", 0.0);
        this->declare_parameter<double>("robot_mass", 0.0);
        this->declare_parameter<std::vector<double>>(
            "center_of_buoyancy", std::vector<double>{ 0.0, 0.0, 0.0 }
        );
        this->declare_parameter<double>("qp_epsilon", 0.0);

        ChassisController::Parameters parameters;
        load_parameters(&parameters);

        this->parameter_sub_ =
            std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto parameter_callback = [this](const rcl_interfaces::msg::ParameterEvent& parameter_event){
            if (parameter_event.node != this->get_fully_qualified_name())
                return;    // quit if for another node
            auto controller_parameters = this->controller_->get_param_struct();    // get copy of current parameters
            bool update = false;
            for (const auto& parameter : parameter_event.changed_parameters) {
                const auto name = rclcpp::Parameter::from_parameter_msg(parameter).get_name();
                const auto value = rclcpp::Parameter::from_parameter_msg(parameter);
                auto it = parameter_handlers().find(name);
                if (it == parameter_handlers().end()) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "No transformer found for parameter '%s', skipping.",
                        name.c_str()
                    );
                    continue;
                }
                auto [before, after] = it->second(controller_parameters, value);
                update = true;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Parameter '%s' changed (%s)->(%s)",
                    name.c_str(), before.c_str(), after.c_str()
                );
            }
            if (update)    // update if parameters dirty
                this->controller_->update_parameters(controller_parameters);
        };

        this->parameter_event_callback_handle_ =
            parameter_sub_->add_parameter_event_callback(parameter_callback);

        thruster_interface_ = std::make_shared<T200Interface>(
            "can0", std::array<unsigned int, N_MOTORS>{ 0x411, 0x412, 0x413, 0x414, 0x415, 0x416, 0x417, 0x418 }
        );
        controller_ = std::make_unique<ChassisController>(parameters, thruster_interface_, 100000);

        // setup motor publishers for sim
        for (int i = 0; i < N_MOTORS; i++) {
            sim_motor_pubs_[i] =
                this->create_publisher<std_msgs::msg::Float64>(
                    "/sim/motor_forces/m_" + std::to_string(i), reliable_profile
                );
        }

        // setup subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", volatile_profile,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
                controller_->update_current_state(*msg);
            }
        );

        cmd_vel_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", volatile_profile,
                [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
                    controller_->update_desired_state(*msg);
                }
            );

        state_sub_ = this->create_subscription<guppy_msgs::msg::State>(
            "/state", volatile_profile,
            [this](const guppy_msgs::msg::State::ConstSharedPtr& msg) {
                this->state_callback(*msg);
            }
        );

        reset_service_ = this->create_service<guppy_msgs::srv::SetHoldPose>(
            "reset_holding_pose",
            [this](
                const std::shared_ptr<guppy_msgs::srv::SetHoldPose::Request> request,
                std::shared_ptr<guppy_msgs::srv::SetHoldPose::Response>      response
            ) {
                this->controller_->reset_holding_pose(request);
                (void)response;
            },
            reliable_profile
        );

        timer_ = this->create_wall_timer(
            10ms,
            [this] {
                auto thrusts = this->controller_->get_motor_thrusts();
                for (int i = 0; i < 8; i++) {
                    std_msgs::msg::Float64 thrust;
                    thrust.data = (double)thrusts[i];
                    this->sim_motor_pubs_[i].get()->publish(thrust);
                }
            }
        );    // publishes sim motor thrust every 10 milliseconds

        RCLCPP_INFO(
            this->get_logger(),
            "Setup parameters, thrust publishers, and subscribers."
        );

        bool controller_debug = false;
        this->declare_parameter("controller_debug", false);
        this->get_parameter("controller_debug", controller_debug);

        this->controller_->start(controller_debug);
    }
private:
    void state_callback(const guppy_msgs::msg::State& msg) {
        if (msg.state == guppy_msgs::msg::State::DISABLED)
            this->thruster_interface_->set_enabled(false);
        else
            this->thruster_interface_->set_enabled(true);
        if (msg.state == guppy_msgs::msg::State::NAV)
            this->controller_->enable_pose_pid(true);
        else
            this->controller_->enable_pose_pid(false);
    }

    // helper to get motor coefficients from a 5 x N matrix of motor positions
    template <int N>
    static Eigen::Matrix<double, 6, N> to_motor_coefficients(const std::vector<double>& flat5N) {
        Eigen::Matrix<double, 6, N> M;
        for (int i = 0; i < N; ++i) {
            const double x     = flat5N[5 * i + 0];
            const double y     = flat5N[5 * i + 1];
            const double z     = flat5N[5 * i + 2];
            const double phi   = flat5N[5 * i + 3];
            const double theta = flat5N[5 * i + 4];
            M.col(i) = get_single_motor_coefficients(x, y, z, phi, theta);
        }
        return M;
    }

    // converts a vector<double> to a string
    static std::string vec_to_str(const std::vector<double>& vec) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            if (i)
                oss << ", ";
            oss << vec[i];
        }
        oss << "]";
        return oss.str();
    }

    // converts any eigen type to a string
    template <typename T>
    static std::string eigen_to_str(
        const Eigen::MatrixBase<T>& matrix, const Eigen::IOFormat& format
    ) {
        std::stringstream stream;
        stream << matrix.format(format);
        return stream.str();
    }

    // creates an eigen vector from a vector
    template <int N>
    static Eigen::Matrix<double, N, 1>
        to_eigen_vec(const std::vector<double>& vec) {
        if (vec.size() != N)
            RCLCPP_ERROR(rclcpp::get_logger("control_node"), "bad vector parameter passed to controller!");
        return Eigen::Map<const Eigen::Matrix<double, N, 1>>(vec.data());
    }

    // creates an eigen matrix from a flattened vector
    template <int R, int C>
    static Eigen::Matrix<double, R, C>
        to_eigen_matrix(const std::vector<double>& vec) {
        if (vec.size() != R * C)
            RCLCPP_ERROR(rclcpp::get_logger("control_node"), "Bad matrix parameter passed to controller!");
        return Eigen::Map<const Eigen::Matrix<double, R, C, Eigen::RowMajor>>(
            vec.data()
        );
    }

    // formatting for eigen matrices/vectors
    inline static const Eigen::IOFormat LineFormat =
        Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
    inline static const Eigen::IOFormat InlineFormat =
        Eigen::IOFormat(3, 0, ", ", "", "", "");

    template<typename Parameters, typename Member, typename Transform, typename Format>
    struct ParameterDescriptor {
        std::string_view name;
        Member Parameters::* member;
        Transform transform;
        Format to_string;
    };

    template<typename Parameters, typename Member, typename Transform, typename Format>
    ParameterDescriptor(std::string_view, Member Parameters::*, Transform, Format)
        -> ParameterDescriptor<Parameters, Member, Transform, Format>;

    constexpr static auto parameters = std::tuple{
        ParameterDescriptor{
            "motor_positions",
            &ChassisController::Parameters::motor_coefficients,
            [](const rclcpp::Parameter& parameter) {
                return to_motor_coefficients<N_MOTORS>(
                    parameter.as_double_array()
                );
            },
            [](const Eigen::Matrix<double, 6, N_MOTORS>& member) -> std::string {
                return eigen_to_str(member, LineFormat);
            }
        },
        ParameterDescriptor{
            "motor_lower_bounds",
            &ChassisController::Parameters::motor_lower_bounds,
            [](const rclcpp::Parameter& value) {
                return to_eigen_vec<N_MOTORS>(value.as_double_array());
            },
            [](const Eigen::Matrix<double, N_MOTORS, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "motor_upper_bounds",
            &ChassisController::Parameters::motor_upper_bounds,
            [](const rclcpp::Parameter& parameter) {
                return to_eigen_vec<N_MOTORS>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, N_MOTORS, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "axis_weight_matrix",
            &ChassisController::Parameters::axis_weight_matrix,
            [](const rclcpp::Parameter& value) {
                return to_eigen_matrix<6, 6>(value.as_double_array());
            },
            [](const Eigen::Matrix<double, 6, 6>& member) -> std::string {
                return eigen_to_str(member, LineFormat);
            }
        },
        ParameterDescriptor{
            "pid_gains_vel_linear",
            &ChassisController::Parameters::pid_gains_vel_linear,
            [](const rclcpp::Parameter& value) {
                  return value.as_double_array();
            },
            [](const std::vector<double>& member) -> std::string {
                return vec_to_str(member);
            }
        },
        ParameterDescriptor{
            "pid_gains_vel_angular",
            &ChassisController::Parameters::pid_gains_vel_angular,
            [](const rclcpp::Parameter& value) {
                return value.as_double_array();
            },
            [](const std::vector<double>& member) -> std::string {
                return vec_to_str(member);
            }
        },
        ParameterDescriptor{
            "pid_gains_pose_linear",
            &ChassisController::Parameters::pid_gains_pose_linear,
            [](const rclcpp::Parameter& value) {
                return value.as_double_array();
            },
            [](const std::vector<double>& member) -> std::string {
                return vec_to_str(member);
            }
        },
        ParameterDescriptor{
            "pid_gains_pose_angular",
            &ChassisController::Parameters::pid_gains_pose_angular,
            [](const rclcpp::Parameter& parameter) {
                return parameter.as_double_array();
            },
            [](const std::vector<double>& member) -> std::string {
                return vec_to_str(member);
            }
        },
        ParameterDescriptor{
            "pose_lock_deadband",
            &ChassisController::Parameters::pose_lock_deadband,
            [](const rclcpp::Parameter& parameter){
                return to_eigen_vec<6>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, 6, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "drag_coefficients",
            &ChassisController::Parameters::drag_coefficients,
            [](const rclcpp::Parameter& parameter){
                return to_eigen_vec<6>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, 6, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "drag_areas",
            &ChassisController::Parameters::drag_areas,
            [](const rclcpp::Parameter& parameter){
                return to_eigen_vec<6>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, 6, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "drag_effect_matrix",
            &ChassisController::Parameters::drag_effect_matrix,
            [](const rclcpp::Parameter& parameter){
                return to_eigen_matrix<6, 6>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, 6, 6>& member) -> std::string {
                return eigen_to_str(member, LineFormat);
            }
        },
        ParameterDescriptor{
            "water_density",
            &ChassisController::Parameters::water_density,
            [](const rclcpp::Parameter& parameter){
                return parameter.as_double();
            },
            [](double member) -> std::string {
                return std::to_string(member);
            }
        },
        ParameterDescriptor{
            "robot_volume",
            &ChassisController::Parameters::robot_volume,
            [](const rclcpp::Parameter& parameter){
                return parameter.as_double();
            },
            [](double member) -> std::string {
                return std::to_string(member);
            }
        },
        ParameterDescriptor{
            "robot_mass",
            &ChassisController::Parameters::robot_mass,
            [](const rclcpp::Parameter& parameter){
                return parameter.as_double();
            },
            [](double member) -> std::string {
                return std::to_string(member);
            }
        },
        ParameterDescriptor{
            "center_of_buoyancy",
            &ChassisController::Parameters::center_of_buoyancy,
            [](const rclcpp::Parameter& parameter){
                return to_eigen_vec<3>(parameter.as_double_array());
            },
            [](const Eigen::Matrix<double, 3, 1>& member) -> std::string {
                return eigen_to_str(member, InlineFormat);
            }
        },
        ParameterDescriptor{
            "qp_epsilon",
            &ChassisController::Parameters::qp_epsilon,
            [](const rclcpp::Parameter& parameter){
                return parameter.as_double();
            },
            [](double member) -> std::string {
                return std::to_string(member);
            }
        }
    };

    using ParameterHandler = std::function<std::pair<std::string, std::string>(ChassisController::Parameters&, const rclcpp::Parameter&)>;

    static const std::unordered_map<std::string, ParameterHandler>& parameter_handlers() {
        static const std::unordered_map<std::string, ParameterHandler> handlers = [] {
            std::unordered_map<std::string, ParameterHandler> map;
            std::apply(
                [&map](const auto&... descriptor) {
                    (
                        map.emplace(
                            descriptor.name,
                            [descriptor](ChassisController::Parameters& parameters, const rclcpp::Parameter& value) {
                                auto& member = parameters.*descriptor.member;
                                std::string before = descriptor.to_string(member);
                                member = descriptor.transform(value);
                                std::string after = descriptor.to_string(member);
                                return std::make_pair(before, after);
                            }
                        ),
                        ...
                    );
                },
                ControlNode::parameters
            );
            return map;
        }();
        return handlers;
    }

    void load_parameters(ChassisController::Parameters* parameters) {
        for (const auto& [name, handler] : parameter_handlers()) {
            if (!this->has_parameter(name)) {
                RCLCPP_WARN(this->get_logger(), "Parameter '%s' not set, skipping.", name.c_str());
                continue;
            }
            handler(*parameters, this->get_parameter(name));
        }
    }

    /*
      @brief get the motor coefficients from a thruster's pose
      @param x the x loctaion in meters of the thruster
      @param y the y loctaion in meters of the thruster
      @param z the z loctaion in meters of the thruster
      @param phi the phi of the thruster in spherical coordinates (degrees,
      positive up)
      @param theta the theta of the thruster in spherical coordinates (degrees)
    */
    static Eigen::Matrix<double, 6, 1> get_single_motor_coefficients(
        double x, double y, double z, double phi, double theta
    ) {
        Eigen::Matrix<double, 6, 1> out;
        // convert to rads
        double p = (90 - phi) * (M_PI / 180);
        double t = (theta) * (M_PI / 180);
        // calculate to reuse
        double sinp = sin(p);
        double sint = sin(t);
        double cost = cos(t);
        double cosp = cos(p);
        out << sinp * cost, sinp * sint, cosp, (z * sinp * sint) - (y * cosp),
            (x * cosp) - (z * sinp * cost),
            (y * sinp * cost) - (x * sinp * sint);
        return out;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
