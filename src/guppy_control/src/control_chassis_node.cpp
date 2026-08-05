#include <rclcpp/subscription.hpp>
#include "rclcpp/rclcpp.hpp"
#include "guppy_control/chassis_controller.hpp"
#include "guppy_control/t200_interface.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "guppy_msgs/srv/set_hold_pose.hpp"
#include "guppy_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using namespace t200_interface;
using namespace chassis_controller;

class ControlChassis : public rclcpp::Node {
public:
  ControlChassis() : Node("control_chassis") {

    // declare parameters

    this->declare_parameter<std::vector<double>>("motor_positions", std::vector<double>(5 * N_MOTORS, 0.0)); // flattened 6xN
    this->declare_parameter<std::vector<double>>("motor_lower_bounds", std::vector<double>(N_MOTORS, 0.0));
    this->declare_parameter<std::vector<double>>("motor_upper_bounds", std::vector<double>(N_MOTORS, 0.0));
    this->declare_parameter<std::vector<double>>("axis_weight_matrix", std::vector<double>(6 * 6, 0.0)); // flattened 6 * 6
    this->declare_parameter<std::vector<double>>("pid_gains_vel_linear", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_vel_angular", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_pose_linear", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_pose_angular", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pose_lock_deadband", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_coefficients", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_areas", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_effect_matrix", std::vector<double>(6 * 6, 0.0)); // flattened 6x6
    this->declare_parameter<double>("water_density", 0.0);
    this->declare_parameter<double>("robot_volume", 0.0);
    this->declare_parameter<double>("robot_mass", 0.0);
    this->declare_parameter<std::vector<double>>("center_of_buoyancy", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<double>("qp_epsilon", 0.0);

    ChassisController::ChassisControllerParams parameters;

    load_params_(&parameters); // load parameters from configuration file

    this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this); // subscribe to parameter change event

    // parameter callback to update parameters on event

    auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent & parameter_event) {
      if (parameter_event.node != this->get_fully_qualified_name()) return; // quit if for another node

      auto controller_params = this->controller->get_param_struct(); // get copy of current parameters

      auto param_map = get_param_map(&controller_params); // map pointers of parameter to parameter names

      bool update = false;

      for (const auto& param : parameter_event.changed_parameters) {
        const auto name = rclcpp::Parameter::from_parameter_msg(param).get_name();
        const auto value = rclcpp::Parameter::from_parameter_msg(param);

        // get entries (from maps below) with values of lambda/pointer based on the parameter name key
        auto pointer_it = param_map.find(name); // get pointer to parameter member in struct (pointer_it->second refers to value)
        auto transformer_it = transformers.find(name); // get transformer lambda to change simple vector to complex eigen type
        auto printer_it = printers.find(name); // get lambda for formatting complex type into string

        std::visit([&](auto *pointer, auto& transformer) { // call lambda with variant types (map<string, ParameterTypes> and PaarameterPointerTypes)
          if (transformer_it == transformers.end()) {
            RCLCPP_INFO(this->get_logger(), "No transformer found for parameter '%s', skipping.", name.c_str());
            return;
          }

          using P = std::remove_cv_t<std::remove_reference_t<decltype(*pointer)>>; // get type at pointer/reference and strip const/volatile
          using R = std::remove_cv_t<std::remove_reference_t<std::invoke_result_t<decltype(transformer), const rclcpp::Parameter&>>>; // final result of calling lambda transformer, ie. transformed type

          if constexpr (std::is_same_v<P, R>) { // ensure transformed type and type at pointer are the same (so you don't set bad data)
            std::string before, after;
            before = printer_it->second(static_cast<const void*>(pointer)); // ignore type of pointer as you pass to printer to format for debugging

            auto tmp = transformer(value); // store transformed value (from double vector to actual parameter type ie. Eigen::Matrix) based on parameter name

            after = printer_it->second(static_cast<const void*>(&tmp)); // ignore type and format result for debugging

            *pointer = std::move(tmp); // fast copy for vector

            update = true; // parameter struct is dirty and should be updated

            RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed (%s)->(%s)", name.c_str(), before.c_str(), after.c_str());
          }
        }, pointer_it->second, transformer_it->second); // pass values into visit, must belong to defined variants
      }

      if (update) // update if parameters dirty
        this->controller->update_parameters(controller_params);
    };
    this->param_event_callback_handle_ = param_subscriber_->add_parameter_event_callback(param_callback);

    thruster_interface = new T200Interface("can0", {0x411, 0x412, 0x413, 0x414, 0x415, 0x416, 0x417, 0x418});

    controller = new ChassisController(parameters, thruster_interface, 100000);

    // setup motor publishers for sim
    for (int i = 0; i < 8; i++) {
      sim_motor_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/sim/motor_forces/m_" + std::to_string(i), 10);
    }

    // setup subscriptions
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10,std::bind(&ControlChassis::odom_callback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&ControlChassis::cmdvel_callback, this, std::placeholders::_1));
    state_subscription_ = this->create_subscription<guppy_msgs::msg::State>("/state", 10, std::bind(&ControlChassis::state_callback, this, std::placeholders::_1));

    reset_service = this->create_service<guppy_msgs::srv::SetHoldPose>("reset_holding_pose", [&](const std::shared_ptr<guppy_msgs::srv::SetHoldPose::Request> request, std::shared_ptr<guppy_msgs::srv::SetHoldPose::Response> response) {
      controller->reset_holding_pose(request);
    }, 10);


    auto timer_callback = [this]() -> void {
      auto thrusts = controller->get_motor_thrusts();

      for (int i = 0; i < 8; i++) {
        std_msgs::msg::Float32 thrust;
        thrust.data = (double)thrusts[i];
        sim_motor_publishers_[i].get()->publish(thrust);
      }
    };
    timer_ = this->create_wall_timer(10ms, timer_callback); // publishes sim motor thrust every 10 milliseconds

    RCLCPP_INFO(this->get_logger(), "Setup parameters, thrust publishers, and subscribers.");

    bool controller_debug = false;
    this->declare_parameter("controller_debug", false);
    this->get_parameter("controller_debug", controller_debug);

    controller->start(controller_debug);
  }

  ~ControlChassis() {
    controller->stop();
    delete controller;
    delete thruster_interface;
  }

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    controller->update_current_state(msg);
  }

  void cmdvel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    controller->update_desired_state(msg);
  }

  void state_callback(guppy_msgs::msg::State::SharedPtr msg) {
      if (msg->state == guppy_msgs::msg::State::DISABLED) {
          this->thruster_interface->set_enabled(false);
      } else {
          this->thruster_interface->set_enabled(true);
      }
      if (msg->state == guppy_msgs::msg::State::NAV) {
          this->controller->enable_pose_pid(true);
      } else {
          this->controller->enable_pose_pid(false);
      }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_motor_publishers_[8];
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<guppy_msgs::srv::SetHoldPose>::SharedPtr reset_service;


  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_event_callback_handle_;

  using ParameterPointerTypes = std::variant< // expected complex types for parameters
    Eigen::Matrix<double, 6, N_MOTORS>*,
    Eigen::Matrix<double, N_MOTORS, 1>*,
    std::vector<double>*,
    Eigen::Matrix<double, 6, 1>*,
    Eigen::Matrix<double, 6, 6>*,
    double*,
    Eigen::Matrix<double, 3, 1>*
  >;

  using ParameterTypes = std::variant< // expected complex lambdas for transformers
    std::function<Eigen::Matrix<double, 6, N_MOTORS>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>,
    std::function<std::vector<double>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>,
    std::function<double(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 3, 1>(const rclcpp::Parameter&)>
  >;

  // transform simple vector/double types into complex eigen types with a lambda, based on parameter name
  std::unordered_map<std::string, ParameterTypes> transformers = {
    {
      "motor_positions",
      std::function<Eigen::Matrix<double, 6, N_MOTORS>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return ControlChassis::to_motor_coefficients<N_MOTORS>(value.as_double_array());
        }
      )
    },
    {
      "motor_lower_bounds",
      std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<N_MOTORS>(value.as_double_array());
        }
      )
    },
    {
      "motor_upper_bounds",
      std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<N_MOTORS>(value.as_double_array());
        }
      )
    },
    {
      "axis_weight_matrix",
      std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_matrix<6,6>(value.as_double_array());
        }
      )
    },
    {
      "pid_gains_vel_linear",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    {
      "pid_gains_vel_angular",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    {
      "pid_gains_pose_linear",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) { return value.as_double_array();
        }
      )
    },
    {
      "pid_gains_pose_angular",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    {
      "pose_lock_deadband",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    {
      "drag_coefficients",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    {
      "drag_areas",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    {
      "drag_effect_matrix",
      std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_matrix<6,6>(value.as_double_array());
        }
      )
    },
    {
      "water_density",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    {
      "robot_volume",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    {
      "robot_mass",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    {
      "center_of_buoyancy",
      std::function<Eigen::Matrix<double, 3, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<3>(value.as_double_array());
        }
      )
    },
    {
      "qp_epsilon",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    }
  };

  // formatting for eigen matrices/vectors
  inline static const Eigen::IOFormat LineFormat = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
  inline static const Eigen::IOFormat InlineFormat = Eigen::IOFormat(3, 0, ", ", "", "", "");

  // format parameter struct members into strings
  std::unordered_map<std::string, std::function<std::string(const void*)>> printers = {
    {
      "motor_positions",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,N_MOTORS>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "axis_weight_matrix",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,6>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "drag_effect_matrix",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,6>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "motor_lower_bounds",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,N_MOTORS,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "motor_upper_bounds",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,N_MOTORS,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "pose_lock_deadband",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "drag_coefficients",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "drag_areas",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "center_of_buoyancy",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,3,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "pid_gains_vel_linear",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_vel_angular",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_pose_linear",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_pose_angular",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "water_density",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "robot_volume",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "robot_mass",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "qp_epsilon",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    }
  };

  T200Interface* thruster_interface;
  ChassisController* controller;

  // set given parameter struct to current ros2 parameters using ros_node#get_parameter()
  void load_params_(ChassisController::ChassisControllerParams* params) {
    auto param_map = get_param_map(params); // get

    for (auto& [name, pointer_to] : param_map) {
      if (!this->has_parameter(name)) {
        RCLCPP_WARN(this->get_logger(), "Parameter '%s' not set, skipping.", name.c_str());
        continue;
      }

      const auto param = this->get_parameter(name);

      auto transformer_it = transformers.find(name);

      std::visit([&](auto* pointer, auto& transformer) {
        using P = std::remove_cv_t<std::remove_reference_t<decltype(*pointer)>>;
        using R = std::invoke_result_t<decltype(transformer), const rclcpp::Parameter&>;

        if constexpr (std::is_same_v<P, R>)
          *pointer = transformer(param);
      }, pointer_to, transformer_it->second);
    }
  }

  // print current values of a parameter struct
  void print_params_struct_(ChassisController::ChassisControllerParams* params) {
    auto param_map = get_param_map(params); // get pointers to parameter struct members by parameter name

    for (auto& [name, pointer_to] : param_map) {
      const auto printer_it = printers.find(name);

      if (printer_it == printers.end()) { // skip if no formatting lambda
        RCLCPP_WARN(this->get_logger(), "No printer registered for '%s', skipping.", name.c_str());
        continue;
      }

      std::string rendered;
      std::visit(
        [&](auto* pointer) {
          rendered = printer_it->second(static_cast<const void*>(pointer));
        }, pointer_to // pass in pointer
      );

      RCLCPP_INFO(this->get_logger(), "%s: %s", name.c_str(), rendered.c_str());
    }
  }

  // helper that prints current parameters in ros2
  void print_params_(void) {
    ChassisController::ChassisControllerParams params;

    load_params_(&params);

    print_params_struct_(&params);
  }

  // converts a vector<double> to a string
  static std::string vec_to_str(const std::vector<double>& vec) {
    std::ostringstream oss;

    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i) oss << ", ";
      oss << vec[i];
    }

    oss << "]";

    return oss.str();
  }

  // converts any eigen type to a string
  template <typename Derived>
  static std::string eigen_to_str(const Eigen::MatrixBase<Derived>& matrix, const Eigen::IOFormat& format) {
    std::ostringstream oss;

    oss << matrix.format(format);

    return oss.str();
  }

  // creates an eigen vector from a vector
  template <int N>
  static Eigen::Matrix<double, N, 1> to_eigen_vec(const std::vector<double>& vec) {
    if (vec.size() != N) {
      //RCLCPP_ERROR(this->get_logger(), "Bad vector parameter passed to controller!");
      throw std::runtime_error("Bad parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(vec.data());
  }

  // creates an eigen matrix from a flattened vector
  template <int R, int C>
  static Eigen::Matrix<double, R, C> to_eigen_matrix(const std::vector<double>& vec) {
    if (vec.size() != R * C) {
      //RCLCPP_ERROR(this->get_logger(), "Bad matrix parameter passed to controller!");
      throw std::runtime_error("Bad matrxc parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, R, C, Eigen::RowMajor>>(vec.data());
  }

  // helper to get motor coefficients from a 5 x N matrix of motor positions
  template <int N>
  static Eigen::Matrix<double, 6, N> to_motor_coefficients(const std::vector<double>& flat5N) {
    Eigen::Matrix<double, 6, N> M;
    for (int i = 0; i < N; ++i) {
      const double x     = flat5N[5*i + 0];
      const double y     = flat5N[5*i + 1];
      const double z     = flat5N[5*i + 2];
      const double phi   = flat5N[5*i + 3];
      const double theta = flat5N[5*i + 4];
      M.col(i) = get_single_motor_coefficients(x, y, z, phi, theta);
    }

    return M;
  }

  // maps pointers to members of a parameter struct to the names of the parameters
  std::unordered_map<std::string, ParameterPointerTypes> get_param_map(ChassisController::ChassisControllerParams* params) {
    return {
      { "motor_positions",        &params->motor_coefficients },
      { "motor_lower_bounds",     &params->motor_lower_bounds },
      { "motor_upper_bounds",     &params->motor_upper_bounds },
      { "axis_weight_matrix",     &params->axis_weight_matrix },
      { "pid_gains_vel_linear",   &params->pid_gains_vel_linear },
      { "pid_gains_vel_angular",  &params->pid_gains_vel_angular },
      { "pid_gains_pose_linear",  &params->pid_gains_pose_linear },
      { "pid_gains_pose_angular", &params->pid_gains_pose_angular },
      { "pose_lock_deadband",     &params->pose_lock_deadband },
      { "drag_coefficients",      &params->drag_coefficients },
      { "drag_areas",             &params->drag_areas },
      { "drag_effect_matrix",     &params->drag_effect_matrix },
      { "water_density",          &params->water_density },
      { "robot_volume",           &params->robot_volume },
      { "robot_mass",             &params->robot_mass },
      { "center_of_buoyancy",     &params->center_of_buoyancy },
      { "qp_epsilon",             &params->qp_epsilon }
    };
  }

  /*
    @brief get the motor coefficients from a thruster's pose
    @param x the x loctaion in meters of the thruster
    @param y the y loctaion in meters of the thruster
    @param z the z loctaion in meters of the thruster
    @param phi the phi of the thruster in spherical coordinates (degrees, positive up)
    @param theta the theta of the thruster in spherical coordinates (degrees)
  */
  static Eigen::Matrix<double, 6, 1> get_single_motor_coefficients(double x, double y, double z, double phi, double theta) {
    Eigen::Matrix<double, 6, 1> out;

    // convert to rads
    double p = (90 - phi) * (M_PI / 180);
    double t = (theta) * (M_PI / 180);
    // calculate to reuse
    double sinp = sin(p);
    double sint = sin(t);
    double cost = cos(t);
    double cosp = cos(p);

    out <<\
        sinp * cost,                  \
        sinp * sint,                  \
        cosp,                         \
        (z*sinp*sint) - (y*cosp),     \
        (x*cosp) - (z*sinp*cost),     \
        (y*sinp*cost) - (x*sinp*sint);
    return out;
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ControlChassis>());

  rclcpp::shutdown();

  return 0;
}
