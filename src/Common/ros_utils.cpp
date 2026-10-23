#include "omx/ros_utils.hpp"

#include <filesystem>
#include <vector>

#include "omx/dynamixel_io.hpp"

namespace omx
{

rclcpp::NodeOptions motorParamsNodeOptions(int argc, char ** argv, const std::string & node_name)
{
  rclcpp::NodeOptions opts;
  const std::string cfg = motorYamlPath();
  if (std::filesystem::exists(cfg)) {
    std::vector<std::string> args = {"--ros-args"};
    bool in_ros_args = false;
    for (int i = 1; i < argc; ++i) {
      const std::string a(argv[i]);
      if (a == "--ros-args") { in_ros_args = true; continue; }
      if (in_ros_args) args.push_back(a);
    }
    args.push_back("--params-file");
    args.push_back(cfg);
    opts.arguments(args);
    opts.use_global_arguments(false);
    RCLCPP_INFO(rclcpp::get_logger(node_name), "motorXM430W350T_params auto-cargado: %s", cfg.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger(node_name),
      "motorXM430W350T_params no encontrado: %s — usando defaults del código.", cfg.c_str());
  }
  return opts;
}

bool openCsv(std::ofstream & csv, const std::string & path, const std::string & header,
             rclcpp::Logger logger)
{
  const auto dir = std::filesystem::path(path).parent_path();
  if (!dir.empty()) std::filesystem::create_directories(dir);
  csv.open(path);
  if (!csv.is_open()) {
    RCLCPP_ERROR(logger, "No se pudo crear el CSV: %s", path.c_str());
    return false;
  }
  csv << header << '\n';
  RCLCPP_INFO(logger, "CSV: %s", path.c_str());
  return true;
}

JointStateListener::JointStateListener(rclcpp::Node & node, const std::string & topic)
{
  sub_ = node.create_subscription<sensor_msgs::msg::JointState>(
    topic, 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      last_ = msg;
      new_msg_ = true;
    });
}

void JointStateListener::read(Vec4 & q, Vec4 & dq) const
{
  static const std::array<std::string, NUM_JOINTS> names = {"joint1", "joint2", "joint3", "joint4"};
  q.setZero();
  dq.setZero();
  if (!last_) return;
  const auto & js = *last_;
  for (int j = 0; j < NUM_JOINTS; ++j) {
    for (std::size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == names[j]) {
        if (i < js.position.size()) q[j]  = js.position[i];
        if (i < js.velocity.size()) dq[j] = js.velocity[i];
        break;
      }
    }
  }
}

}  // namespace omx
