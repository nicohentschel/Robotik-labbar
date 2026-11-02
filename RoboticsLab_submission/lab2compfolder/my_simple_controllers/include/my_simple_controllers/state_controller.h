#ifndef SIMPLE_STATE_CONTROLLER_H
#define SIMPLE_STATE_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <memory>
#include <string>
#include <vector>

//auto-gernerated by generate_parameters library
#include "my_simple_controller_parameters.hpp"

namespace my_simple_controllers {

  class StateController : public controller_interface::ControllerInterface {
    public:
        //ROS2 control requires that we implement these functions:
        //called during configuration of command interfaces
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        //called during configuration of state interfaces
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        //calls init on base class
        controller_interface::CallbackReturn on_init() override;
        //configure parameters
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        //clears state and will start control after this
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;
        //clean-up back to a state from which we can re-start the controller
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        //called once every cycle to update --> use realtime tools within this
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
    private:
        
        // Parameters from ROS for controller
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;
        
        //store the robot description as a URDF string
        std::string urdf_; 

        //read robot description from parameter server
        bool getRobotDescriptionFromServer();
        
        //names of all joints to monitor
        std::vector<std::string> joint_names_;
        unsigned int n_joints_;
        
        template<typename T>
          using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
        //reference to the state interface
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

        const std::vector<std::string> allowed_state_interface_types_ = {
          hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY,
          hardware_interface::HW_IF_ACCELERATION
         // hardware_interface::HW_IF_EFFORT,
        };

        //fk with kdl
        KDL::Tree tree;
        KDL::Chain chain;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fkSolver;
        KDL::JntArray q;
        std::vector<size_t> chainToJointIndex;   //chain joint i -> index in joint_names_

        //tf publishing, realtime safe
        using TFMsg = tf2_msgs::msg::TFMessage;
        rclcpp::Publisher<TFMsg>::SharedPtr tfPublisher;
        std::unique_ptr<realtime_tools::RealtimePublisher<TFMsg>> rtTfPublisher;

        //throttling
        rclcpp::Duration publishPeriod{0, 0};
        rclcpp::Time lastPublishTime;
        bool firstUpdate = true;

  };


};

#endif
