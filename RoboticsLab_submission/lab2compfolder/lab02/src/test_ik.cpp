#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <stdexcept>
#include <vector>
#include <random>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lab02/srv/solve_ik.hpp"      

#include "kdl_parser/kdl_parser.hpp"
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <urdf/model.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

//srv::
using SolveIK = lab02::srv::SolveIK;
//class
class TestIK : public rclcpp::Node
{

public:
    TestIK() : Node("test_ik")
    {
    
    urdfPath = declare_parameter<std::string>("urdfPath", "/tmp/rrbot.urdf");
    lambda = declare_parameter<double>("lambda", 0.1);
    maxiter = declare_parameter<int>("maxIterations", 1000);
    posTol = declare_parameter<double>("posTol", 0.0001);
    rotTol = declare_parameter<double>("rotTol", 0.001);
    damping = declare_parameter<double>("damping", 0.0);

    // urdf->kdltree

    std::ifstream file(urdfPath);

    std::stringstream buf;
    buf << file.rdbuf();
    std::string urdf = buf.str();
    if (!kdl_parser::treeFromString(urdf, tree)) {
        throw std::runtime_error("Failed URDF .> KDL tree");
    }
    numberOfJoints = tree.getNrOfJoints();
    RCLCPP_INFO(get_logger(), "kdl tree: %u jnts", numberOfJoints);
    // TODO C: joint names + limits, in KDL's order
    urdf::Model model;
    model.initString(urdf);

    jointNames.resize(numberOfJoints);
    lower.resize(numberOfJoints);
    upper.resize(numberOfJoints);

    for (auto seg : tree.getSegments()) {
      KDL::Joint joint = seg.second.segment.getJoint();
      if (joint.getType() == KDL::Joint::None) {
        continue;                      //fixed joint -> sskip it
      }
      int i = seg.second.q_nr;         //kdls index for this joint
      jointNames[i] = joint.getName();
      lower[i] = model.getJoint(joint.getName())->limits->lower;
      upper[i] = model.getJoint(joint.getName())->limits->upper;
    }

    for (unsigned int i = 0; i < numberOfJoints; i++) {
      RCLCPP_INFO(get_logger(), "%s: [%.2f, %.2f]",
                  jointNames[i].c_str(), lower[i], upper[i]);
    }

    

    jointPublisher = create_publisher<sensor_msgs::msg::JointState>("ik_joint_states", 10);
    service = create_service<SolveIK>("solve_ik", [this](std::shared_ptr<SolveIK::Request> req, std::shared_ptr<SolveIK::Response> res){solve(req, res);});
    RCLCPP_INFO(get_logger(), "test_ik ON, service / solve_ik");
    }


private:
    void solve(std::shared_ptr<SolveIK::Request> req, std::shared_ptr<SolveIK::Response> res)
    {
        RCLCPP_INFO(get_logger(), "Rreq for frame '%s' -> (%.3f, %.3f, %.3f)", req->frame_name.c_str(), req->target_pose.position.x, req->target_pose.position.y,
                                    req->target_pose.position.z);

                //solvers
        KDL::TreeFkSolverPos_recursive fkSolver(tree);
        KDL::TreeJntToJacSolver jacSolver(tree);

        //random initial guess within joint limits
        std::random_device rd;
        std::mt19937 gen(rd());
        KDL::JntArray q(numberOfJoints);
        for (unsigned int i = 0; i < numberOfJoints; i++) {
            std::uniform_real_distribution<double> dist(lower[i], upper[i]);
            q(i) = dist(gen);
        }
        std::string guessStr;
        for (unsigned int i = 0; i < numberOfJoints; i++) {
            guessStr += std::to_string(q(i)) + " ";
        }
        RCLCPP_INFO(get_logger(), "initial guess = [ %s]", guessStr.c_str());

        //goal pose Tg as eigen transform
        Eigen::Vector3d goalPos(req->target_pose.position.x, req->target_pose.position.y, req->target_pose.position.z);
        Eigen::Quaterniond goalRot(req->target_pose.orientation.w, req->target_pose.orientation.x,
                                   req->target_pose.orientation.y, req->target_pose.orientation.z);
        Eigen::Affine3d Tg = Eigen::Translation3d(goalPos) * goalRot.normalized();

        //the loop
                //ik loop
        KDL::Frame frame;
        KDL::Jacobian jac(numberOfJoints);
        sensor_msgs::msg::JointState jointMsg;
        jointMsg.name = jointNames;
        jointMsg.position.resize(numberOfJoints);
        bool converged = 0;

        for (int iter = 0; iter < maxiter; iter++) {
            //fk -> current pose Tc
            fkSolver.JntToCart(q, frame, req->frame_name);
            double x, y, z, w;
            frame.M.GetQuaternion(x, y, z, w);
            Eigen::Vector3d curPos(frame.p.x(), frame.p.y(), frame.p.z());
            Eigen::Quaterniond curRot(w, x, y, z);
            Eigen::Affine3d Tc = Eigen::Translation3d(curPos) * curRot;

            //error Tc -> Tg
            Eigen::Vector3d posError = Tg.translation() - Tc.translation();
            Eigen::AngleAxisd rotError(Tg.linear() * Tc.linear().transpose());

            //check convergence
            if (posError.norm() < posTol && rotError.angle() < rotTol) {
                converged = 1;
                RCLCPP_INFO(get_logger(), "converged after %d iter", iter);
                break;
            }

            //twist v, dt = 1
            Eigen::VectorXd v(6);
            v << posError, rotError.angle() * rotError.axis();

            //jacobian at current q
            jacSolver.JntToJac(q, jac, req->frame_name);
            Eigen::MatrixXd J = jac.data;

            //J^+ : pseudoinverse or damped least squares
            Eigen::MatrixXd Jpinv;
            if (damping > 0) {
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(numberOfJoints, numberOfJoints);
                Jpinv = (J.transpose() * J + damping * damping * I).inverse() * J.transpose();
            } else {
                Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
            }

            //q = q + lambda * J^+ * v
            q.data = q.data + lambda * Jpinv * v;

            //publish current guess
            for (unsigned int i = 0; i < numberOfJoints; i++) {
                jointMsg.position[i] = q(i);
            }
            jointMsg.header.stamp = now();
            jointPublisher->publish(jointMsg);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }

        //returning result
        for (unsigned int i = 0; i < numberOfJoints; i++) {
            jointMsg.position[i] = q(i);
        }
        jointMsg.header.stamp = now();
        jointPublisher->publish(jointMsg);

        
        res->solution = jointMsg;
        res->success = converged;
        if (!converged) {
            RCLCPP_WARN(get_logger(), "no convergence after %d iter", maxiter);
        }
        std::string solStr;
        for (unsigned int i = 0; i < numberOfJoints; i++) {
            solStr += std::to_string(q(i)) + " ";
        }
        RCLCPP_INFO(get_logger(), "solution = [ %s]", solStr.c_str());
    }
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher;
    rclcpp::Service<SolveIK>::SharedPtr service;

    std::string urdfPath;
    double lambda;
    int maxiter;
    double posTol; //position tolerance
    double rotTol; //rotation tolerance
    double damping; //0 = plain pseudoinverse
    KDL::Tree tree;
    unsigned int numberOfJoints;
    std::vector<std::string> jointNames;
    std::vector<double> lower;
    std::vector<double> upper;


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestIK>());
    rclcpp::shutdown();
    return 0;
}