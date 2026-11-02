#include<my_simple_controllers/state_controller.h>
#include <pluginlib/class_list_macros.hpp>  // to allow the controller to be loaded as a plugin
#include <unistd.h>  // usleep()
#include <controller_interface/controller_interface_base.hpp>
#include "controller_interface/helpers.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>

#include<chrono>
using namespace std::chrono_literals;

namespace my_simple_controllers {


////////////////////////////////////////////////////////////////////////////////
//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//-  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
////////////////////////////////////////////////////////////////////////////////
//
//              R O S   C O N T R O L L E R   I N T E R F A C E
//
////////////////////////////////////////////////////////////////////////////////

//=====================================================================================
controller_interface::CallbackReturn StateController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "State controller initializing");
  
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  //here read robot description from parameter server
  if(!getRobotDescriptionFromServer()) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not fetch robot_description from server");
    return CallbackReturn::ERROR;
  }

  //read other parameters as required from config
  return CallbackReturn::SUCCESS;
}
        
bool StateController::getRobotDescriptionFromServer() {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
  while (!param_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
  }

  auto parameters = param_client->get_parameters({ "robot_description" });
  for (auto& parameter : parameters)
  {
    if (parameter.get_name() == "robot_description")
    {
      urdf_ = parameter.value_to_string();
      break;
    }
  }
  return true;
}

//called during configuration of command interfaces
controller_interface::InterfaceConfiguration StateController::command_interface_configuration() const
{
  //RCLCPP_INFO(get_node()->get_logger(), "State controller does not claim any command interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

//called during configuration of state interfaces
controller_interface::InterfaceConfiguration StateController::state_interface_configuration() const 
{
  //RCLCPP_INFO(get_node()->get_logger(), "State controller claiming state interfaces");
  
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::ALL;
  return conf; 
}

//configure parameters
controller_interface::CallbackReturn StateController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {

  (void) previous_state; //clears warning 
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(get_node()->get_logger(), "State controller configuring");

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(logger, "'joints' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  joint_names_ =  params_.joints;
  // get degrees of freedom
  n_joints_ = joint_names_.size();

  if (params_.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_state_interface_.resize(allowed_state_interface_types_.size());

  auto has_ifce_type = [](const std::vector<std::string> & interface_type_list, const std::string & interface_type) {
    return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
  };
  bool has_pos = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
  bool has_vel = has_ifce_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);

  if(has_pos && has_vel) {
    RCLCPP_INFO(logger,"We have both position and velocity state info, all good.\n");
  } else {
    RCLCPP_ERROR(logger, "'state_interfaces' need to contain both position and velocity");
    return CallbackReturn::FAILURE;
  }
  
  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "State interfaces are [%s].",
    get_interface_list(params_.state_interfaces).c_str());

  //parse urdf string -> kdl tree -> chain base -> tip
  if (urdf_.empty()) {
    RCLCPP_ERROR(logger, "robot_description is empty");
    return CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromString(urdf_, tree)) {
    RCLCPP_ERROR(logger, "Failed URDF -> KDL tree");
    return CallbackReturn::ERROR;
  }
  if (!tree.getChain(params_.base_frame, params_.tip_frame, chain)) {
    RCLCPP_ERROR(logger, "No chain %s -> %s", params_.base_frame.c_str(), params_.tip_frame.c_str());
    return CallbackReturn::ERROR;
  }

  //for each moving joint in the chain, find its index in joint_names_
  //(the state interfaces are ordered like joint_names_)
  chainToJointIndex.clear();
  for (auto segment : chain.segments) {
    KDL::Joint joint = segment.getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;   //fixed joint
    }
    auto it = std::find(joint_names_.begin(), joint_names_.end(), joint.getName());
    if (it == joint_names_.end()) {
      RCLCPP_ERROR(logger, "chain joint '%s' is not in the 'joints' parameter", joint.getName().c_str());
      return CallbackReturn::ERROR;
    }
    chainToJointIndex.push_back(std::distance(joint_names_.begin(), it));
  }

  q = KDL::JntArray(chain.getNrOfJoints());
  fkSolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);   //member, so update() doesnt allocate
  RCLCPP_INFO(logger, "kdl chain %s -> %s: %u segments, %u jnts", params_.base_frame.c_str(),
              params_.tip_frame.c_str(), chain.getNrOfSegments(), chain.getNrOfJoints());

  //tf publisher, realtime safe version (a normal tf broadcaster can block inside update)
  tfPublisher = get_node()->create_publisher<TFMsg>("/tf", rclcpp::QoS(100));
  rtTfPublisher = std::make_unique<realtime_tools::RealtimePublisher<TFMsg>>(tfPublisher);

  //set up the message once, so update() doesnt allocate
  rtTfPublisher->lock();
  rtTfPublisher->msg_.transforms.resize(1);
  rtTfPublisher->msg_.transforms[0].header.frame_id = params_.base_frame;
  rtTfPublisher->msg_.transforms[0].child_frame_id = params_.fk_frame;
  rtTfPublisher->msg_.transforms[0].transform.rotation.w = 1.0;
  rtTfPublisher->unlock();

  //throttle: publish at state_publish_rate hz, not every cycle
  publishPeriod = rclcpp::Duration::from_seconds(1.0 / params_.state_publish_rate);
  firstUpdate = true;

  RCLCPP_INFO(logger, "State controller configured");
  return CallbackReturn::SUCCESS;

}

//clears state and will start control after this
controller_interface::CallbackReturn StateController::on_activate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "State controller activating");

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  //order all joints in storage
  for (const auto & interface : params_.state_interfaces)
  {
    auto it =
      std::find(allowed_state_interface_types_.begin(), allowed_state_interface_types_.end(), interface);
    auto index = std::distance(allowed_state_interface_types_.begin(), it);
    if (it==allowed_state_interface_types_.end()) {
      RCLCPP_ERROR(logger, "Could not find interface of type %s in allowed interfaces",interface.c_str());
      return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %u '%s' state interfaces, got %lu.", n_joints_,
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    } 
  }

  RCLCPP_INFO(logger, "State controller activated");
  return CallbackReturn::SUCCESS;
}

//clean-up back to a state from which we can start the controller
controller_interface::CallbackReturn StateController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state) {

  (void)previous_state;
  RCLCPP_INFO(get_node()->get_logger(), "State controller deactivating");
  return CallbackReturn::SUCCESS;
}

//called once every cycle to update --> use realtime tools within this
controller_interface::return_type StateController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) {

  (void)period;

  //throttle: only compute + publish every publishPeriod
  if (!firstUpdate && (time - lastPublishTime) < publishPeriod) {
    return controller_interface::return_type::OK;
  }

  //read joint positions, joint_state_interface_[0] = position interfaces
  //(HW_IF_POSITION is first in allowed_state_interface_types_), ordered like joint_names_
  for (unsigned int i = 0; i < chainToJointIndex.size(); i++) {
    q(i) = joint_state_interface_[0][chainToJointIndex[i]].get().get_value();
  }

  //fk base -> tip
  KDL::Frame frame;
  if (fkSolver->JntToCart(q, frame) < 0) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "fk failed");
    return controller_interface::return_type::OK;
  }

  //fk result -> tf message, publish (skip if the publisher is busy)
  if (rtTfPublisher && rtTfPublisher->trylock()) {
    auto & tf = rtTfPublisher->msg_.transforms[0];
    tf.header.stamp = time;
    tf.transform.translation.x = frame.p.x();
    tf.transform.translation.y = frame.p.y();
    tf.transform.translation.z = frame.p.z();
    frame.M.GetQuaternion(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
    rtTfPublisher->unlockAndPublish();
  }

  lastPublishTime = time;
  firstUpdate = false;
  return controller_interface::return_type::OK;
} 


}

// make the controller available to the library loader
PLUGINLIB_EXPORT_CLASS(my_simple_controllers::StateController,
    controller_interface::ControllerInterface)
