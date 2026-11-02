#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "kdl_parser/kdl_parser.hpp"
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std::chrono_literals;

class TestFK : public rclcpp::Node
{
public:
    TestFK() : Node("test_fk")
    {
        urdfPath = declare_parameter<std::string>("urdfPath", "/tmp/rrbot.urdf");
        baseFrame = declare_parameter<std::string>("baseFrame", "base_link");
        tipFrame = declare_parameter<std::string>("tipFrame", "tool_link");
        fkFrame = declare_parameter<std::string>("fkFrame", "my_fk");

        //read urdf file to a string
        std::ifstream file(urdfPath);
        std::stringstream buf;
        buf << file.rdbuf();
        std::string urdf = buf.str();

        //urdf -> kdl tree -> chain base -> tip
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(urdf, tree)) {
            throw std::runtime_error("Failed URDF -> KDL tree");
        }
        if (!tree.getChain(baseFrame, tipFrame, chain)) {
            throw std::runtime_error("No chain " + baseFrame + " -> " + tipFrame);
        }

        //names of the moving joints in chain order (skip fixed)
        for (auto seg : chain.segments) {
            if (seg.getJoint().getType() != KDL::Joint::None) {
                jointNames.push_back(seg.getJoint().getName());
            }
        }
        q = KDL::JntArray(chain.getNrOfJoints());
        RCLCPP_INFO(get_logger(), "chain %s -> %s: %u jnts", baseFrame.c_str(), tipFrame.c_str(), chain.getNrOfJoints());

        //transform to publish, identity at start
        transform.header.frame_id = baseFrame;
        transform.child_frame_id = fkFrame;
        transform.transform.rotation.w = 1.0;

        broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer = create_wall_timer(10ms, [this]() {publish();});   //100 hz

        jointSub = create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(msg);});
    }

private:
    void onJointState(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //joint states can come in any order, match by name
        for (unsigned int i = 0; i < jointNames.size(); i++) {
            for (unsigned int k = 0; k < msg->name.size(); k++) {
                if (msg->name[k] == jointNames[i]) {
                    q(i) = msg->position[k];
                }
            }
        }

        //fk base -> tip
        KDL::ChainFkSolverPos_recursive fkSolver(chain);
        KDL::Frame frame;
        if (fkSolver.JntToCart(q, frame) < 0) {
            RCLCPP_WARN(get_logger(), "fk failed");
            return;
        }

        transform.transform.translation.x = frame.p.x();
        transform.transform.translation.y = frame.p.y();
        transform.transform.translation.z = frame.p.z();
        double x, y, z, w;
        frame.M.GetQuaternion(x, y, z, w);
        transform.transform.rotation.x = x;
        transform.transform.rotation.y = y;
        transform.transform.rotation.z = z;
        transform.transform.rotation.w = w;
    }

    void publish()
    {
        transform.header.stamp = now();
        broadcaster->sendTransform(transform);
    }

    std::string urdfPath, baseFrame, tipFrame, fkFrame;
    KDL::Chain chain;
    KDL::JntArray q;
    std::vector<std::string> jointNames;
    geometry_msgs::msg::TransformStamped transform;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestFK>());
    rclcpp::shutdown();
    return 0;
}
