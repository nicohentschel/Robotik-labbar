// Lab 1, Part 3: Forward kinematics with KDL
//
// - reads the path to a URDF file from the parameter "urdf_path"
// - parses it with kdl_parser into a KDL::Tree and extracts the chain
//   base_link -> tool_link
// - publishes base_link -> my_fk to TF at 100 Hz (identity until the first
//   joint state arrives)
// - on every JointState message, computes FK with KDL and updates my_fk

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TestFK : public rclcpp::Node
{
public:
  TestFK()
  : Node("test_fk")
  {
    const auto urdf_path = declare_parameter<std::string>("urdf_path", "/tmp/rrbot.urdf");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tip_frame_ = declare_parameter<std::string>("tip_frame", "tool_link");
    fk_frame_ = declare_parameter<std::string>("fk_frame", "my_fk");

    // --- Read the URDF file into a string -------------------------------
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
      RCLCPP_FATAL(get_logger(), "Could not open URDF file '%s'", urdf_path.c_str());
      throw std::runtime_error("cannot open urdf file");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string urdf = buffer.str();

    // --- Parse into a KDL tree and extract the chain ---------------------
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree)) {
      RCLCPP_FATAL(get_logger(), "Failed to parse URDF into a KDL tree");
      throw std::runtime_error("kdl_parser failed");
    }
    if (!tree.getChain(base_frame_, tip_frame_, chain_)) {
      RCLCPP_FATAL(get_logger(), "No chain from '%s' to '%s'",
                   base_frame_.c_str(), tip_frame_.c_str());
      throw std::runtime_error("getChain failed");
    }

    // Names of the movable joints in chain order (fixed joints are skipped)
    for (const auto & seg : chain_.segments) {
      if (seg.getJoint().getType() != KDL::Joint::None) {
        joint_names_.push_back(seg.getJoint().getName());
      }
    }
    q_ = KDL::JntArray(chain_.getNrOfJoints());
    solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    std::string names;
    for (const auto & n : joint_names_) {names += n + " ";}
    RCLCPP_INFO(get_logger(), "Loaded chain %s -> %s: %u segments, %u joints [ %s]",
                base_frame_.c_str(), tip_frame_.c_str(),
                chain_.getNrOfSegments(), chain_.getNrOfJoints(), names.c_str());

    // --- Transform to publish, identity to begin with ---------------------
    transform_.header.frame_id = base_frame_;
    transform_.child_frame_id = fk_frame_;
    transform_.transform.rotation.w = 1.0;

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(10ms, [this]() {publish();});  // 100 Hz

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(msg);});
  }

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Joint states may come in any order, so match them by name
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
        if (msg->name[k] == joint_names_[i]) {
          q_(i) = msg->position[k];
          break;
        }
      }
    }

    KDL::Frame F;
    if (solver_->JntToCart(q_, F) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "FK solver failed");
      return;
    }

    transform_.transform.translation.x = F.p.x();
    transform_.transform.translation.y = F.p.y();
    transform_.transform.translation.z = F.p.z();
    double x, y, z, w;
    F.M.GetQuaternion(x, y, z, w);
    transform_.transform.rotation.x = x;
    transform_.transform.rotation.y = y;
    transform_.transform.rotation.z = z;
    transform_.transform.rotation.w = w;
  }

  void publish()
  {
    transform_.header.stamp = now();
    broadcaster_->sendTransform(transform_);
  }

  std::string base_frame_, tip_frame_, fk_frame_;
  KDL::Chain chain_;
  KDL::JntArray q_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_;
  geometry_msgs::msg::TransformStamped transform_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestFK>());
  rclcpp::shutdown();
  return 0;
}
