
#include <fstream>      
#include <sstream>      
#include <vector>
#include <cmath>       
#include "rclcpp/rclcpp.hpp"            
#include "kdl_parser/kdl_parser.hpp"     
#include <kdl/tree.hpp>                 
#include <kdl/jntarray.hpp>         
#include <kdl/jacobian.hpp>              
#include <kdl/treejnttojacsolver.hpp>   
#include <Eigen/Dense>               


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_jacobians");

    std::string urdfPath = node->declare_parameter<std::string>("urdfPath", "/tmp/rrbot.urdf");
    std::string tipFrame = node->declare_parameter<std::string>("tipFrame", "tool_link");

    RCLCPP_INFO(node->get_logger(), "urdfPath= %s, tipFrame= %s", urdfPath.c_str(), tipFrame.c_str());

    //todo read urdf file to a string
    std::ifstream file(urdfPath);
    if (!file.is_open()) {
    RCLCPP_FATAL(node->get_logger(), "Could not file '%s'", urdfPath.c_str());
    rclcpp::shutdown();
    return 1;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string urdf = buffer.str();
    //todo build kdltree with kdlparser
        KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree)) {
    RCLCPP_FATAL(node->get_logger(), "Failed parse");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "kdltree: %u joints, %u segments",
            tree.getNrOfJoints(), tree.getNrOfSegments());
    //create kdltreejnttojacsolver
    KDL::TreeJntToJacSolver jacSolver(tree);
    //calc jacobian, test configs
    int joints = tree.getNrOfJoints();
    KDL::JntArray q(joints);
    q(0) = 0;
    q(1) = 0;

    KDL::Jacobian jac(joints);
    if(jacSolver.JntToJac(q, jac, tipFrame) < 0 ) {
        RCLCPP_ERROR(node->get_logger(), "JntToJac fail");
        rclcpp::shutdown();
        return 1;
    }
    //convert to eigen & O
    Eigen::MatrixXd jacobian = jac.data;
    std::stringstream s;
    s << "\n" << jacobian;
    RCLCPP_INFO(node->get_logger(), "q = [%.3f, %.3f], jacobian = %s", q(0), q(1), s.str().c_str());
    //compare with the analytical jacobian

    std::vector<std::vector<double>> configs = {{0,0}, {M_PI/2,0}, {M_PI/4,M_PI/4}, {0.3,-1.2}};
    for( auto & cvar : configs) {
        q(0) = cvar[0];
        q(1) = cvar[1];

        if(jacSolver.JntToJac(q, jac, tipFrame) < 0 ) {
        RCLCPP_ERROR(node->get_logger(), "JntToJac fail");
        rclcpp::shutdown();
        return 1;
    }
    //convert to eigen & O
    Eigen::MatrixXd jacobian = jac.data;
    double s1 = std::sin(q(0)), c1 = std::cos(q(0));
    double s12 = std::sin(q(0) + q(1)), c12 = std::cos(q(0) + q(1));
    double L1 = 0.9, L2 = 1.0;
    Eigen::MatrixXd JAnalytical = Eigen::MatrixXd::Zero(6, joints);
    JAnalytical(0,0) = L1*c1 + L2*c12;
    JAnalytical(0,1) = L2*c12;
    JAnalytical(2,0) = -(L1*s1 + L2*s12);
    JAnalytical(2,1) = -L2*s12;
    //JAnalytical(3,0) = 0;
    //JAnalytical(3,1) = 0;
    JAnalytical(4,0) = 1.0;
    JAnalytical(4,1) = 1.0;

    double error = (jacobian - JAnalytical).norm();
    RCLCPP_INFO(node->get_logger(), "q = [%.3f, %.3f] error = %.3e", q(0), q(1), error);

    }




    rclcpp::shutdown();
    return 0;
}