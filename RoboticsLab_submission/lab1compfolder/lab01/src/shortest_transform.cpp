#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ShortestTransform : public rclcpp::Node
{
public:
    ShortestTransform() : Node("shortest_transform")
    {
        numSteps = declare_parameter<int>("numSteps", 10);
        worldFrame = declare_parameter<std::string>("worldFrame", "world");

        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        //two random frames: 3 euler angles in [-pi, pi] + 3 translations in [0, 1]
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> angle(-M_PI, M_PI);
        std::uniform_real_distribution<double> pos(0.0, 1.0);

        T1 = Eigen::Translation3d(pos(gen), pos(gen), pos(gen)) * eulerToRot(angle(gen), angle(gen), angle(gen));
        T2 = Eigen::Translation3d(pos(gen), pos(gen), pos(gen)) * eulerToRot(angle(gen), angle(gen), angle(gen));

        //transform that brings frame1 to frame2: T1 * T12 = T2
        Eigen::Affine3d T12 = T1.inverse() * T2;
        Eigen::Vector3d t = T12.translation();

        //rotation part as angle-axis (v, a), eigen gives a in [0, pi]
        Eigen::AngleAxisd aa(T12.linear());
        Eigen::Vector3d v = aa.axis();
        double a = aa.angle();

        RCLCPP_INFO(get_logger(), "t = [%.3f, %.3f, %.3f], |t| = %.3f m", t.x(), t.y(), t.z(), t.norm());
        RCLCPP_INFO(get_logger(), "v = [%.3f, %.3f, %.3f], a = %.3f rad (%.1f deg)", v.x(), v.y(), v.z(), a, a * 180.0 / M_PI);

        //steps: frame1, then translation t_i and rotation (v, a_i)
        for (int i = 0; i < numSteps; i++) {
            double s = (numSteps > 1) ? (double)i / (numSteps - 1) : 1.0;
            Eigen::Affine3d Ti = T1 * Eigen::Translation3d(s * t) * Eigen::AngleAxisd(s * a, v);
            steps.push_back(Ti);
        }

        //last step should be frame2
        double posError = (steps.back().translation() - T2.translation()).norm();
        double rotError = (steps.back().linear() - T2.linear()).norm();
        RCLCPP_INFO(get_logger(), "last step vs frame2: pos error = %.2e, rot error = %.2e", posError, rotError);

        timer = create_wall_timer(100ms, [this]() {publishAll();});
    }

private:
    //yaw-pitch-roll (z, y, x) -> rotation
    Eigen::Quaterniond eulerToRot(double yaw, double pitch, double roll)
    {
        return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }

    //eigen transform -> tf message, via quaternion
    geometry_msgs::msg::TransformStamped toMsg(Eigen::Affine3d T, rclcpp::Time stamp, std::string child)
    {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = worldFrame;
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
        rclcpp::Time stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> msgs;
        msgs.push_back(toMsg(T1, stamp, "frame1"));
        msgs.push_back(toMsg(T2, stamp, "frame2"));
        for (unsigned int i = 0; i < steps.size(); i++) {
            msgs.push_back(toMsg(steps[i], stamp, "step_" + std::to_string(i)));
        }
        broadcaster->sendTransform(msgs);
    }

    int numSteps;
    std::string worldFrame;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::TimerBase::SharedPtr timer;
    Eigen::Affine3d T1, T2;
    std::vector<Eigen::Affine3d> steps;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShortestTransform>());
    rclcpp::shutdown();
    return 0;
}
