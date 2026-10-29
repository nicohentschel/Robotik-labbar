#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ShortestTransform : public rclcpp::Node
{
public:
  ShortestTransform()
  : Node("shortest_transform")
  {
    num_steps = declare_parameter<int>("num_steps", 10);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Two random frames
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> ang(-M_PI, M_PI);
    std::uniform_real_distribution<double> pos(0.0, 1.0);

    T1_ = randomTransform(gen, ang, pos);
    T2_ = randomTransform(gen, ang, pos);

    //T12 = T1^-1 * T2
    //T1 * T12 = T2
    const Eigen::Affine3d T12 = T1_.inverse() * T2_;
    const Eigen::Matrix3d R = T12.linear();
    const Eigen::Vector3d t = T12.translation();

    //Rotation part to angle-axis (v, a)
    //Eigen returns the angle in [0, pi]
    const Eigen::AngleAxisd aa(R);
    const Eigen::Vector3d v = aa.axis();
    const double a = aa.angle();

    RCLCPP_INFO(get_logger(), "Translation t = [%.3f, %.3f, %.3f]  |t| = %.3f m",
                t.x(), t.y(), t.z(), t.norm());
    RCLCPP_INFO(get_logger(), "Axis v = [%.3f, %.3f, %.3f]  angle a = %.3f rad (%.1f deg)",
                v.x(), v.y(), v.z(), a, a * 180.0 / M_PI);

    for (int i = 0; i < num_steps; ++i) {
      const double s = (num_steps > 1) ? static_cast<double>(i) / (num_steps - 1) : 1.0;
      const double a_i = s * a;
      const Eigen::Vector3d t_i = s * t;

      // frame1, followed by translation t_i and rotation (v, a_i)
      const Eigen::Affine3d Ti =
        T1_ * Eigen::Translation3d(t_i) * Eigen::AngleAxisd(a_i, v);
      steps_.push_back(Ti);
    }

    const double err_pos = (steps_.back().translation() - T2_.translation()).norm();
    const double err_rot = (steps_.back().linear() - T2_.linear()).norm();
    RCLCPP_INFO(get_logger(), "Check last step vs frame2: pos err = %.2e, rot err = %.2e",
                err_pos, err_rot);

    timer_ = create_wall_timer(100ms, [this]() {publishAll();});
  }

private:
  Eigen::Affine3d randomTransform(
    std::mt19937 & gen,
    std::uniform_real_distribution<double> & ang,
    std::uniform_real_distribution<double> & pos)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() << pos(gen), pos(gen), pos(gen);
    const double yaw = ang(gen), pitch = ang(gen), roll = ang(gen);
    T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return T;
  }

  // Eigen transform -> TF message, going via a quaternion
  geometry_msgs::msg::TransformStamped toMsg(
    const Eigen::Affine3d & T, const rclcpp::Time & stamp,
    const std::string & parent, const std::string & child)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform.translation.x = T.translation().x();
    msg.transform.translation.y = T.translation().y();
    msg.transform.translation.z = T.translation().z();
    Eigen::Quaterniond q(T.linear());
    q.normalize();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    return msg;
  }

  void publishAll()
  {
    const rclcpp::Time stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> msgs;
    msgs.push_back(toMsg(T1_, stamp, world_frame_, "frame1"));
    msgs.push_back(toMsg(T2_, stamp, world_frame_, "frame2"));
    for (size_t i = 0; i < steps_.size(); ++i) {
      msgs.push_back(toMsg(steps_[i], stamp, world_frame_, "step_" + std::to_string(i)));
    }
    broadcaster_->sendTransform(msgs);
  }

  int num_steps;
  std::string world_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Affine3d T1_, T2_;
  std::vector<Eigen::Affine3d> steps_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShortestTransform>());
  rclcpp::shutdown();
  return 0;
}
