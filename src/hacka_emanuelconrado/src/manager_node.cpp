#include "hacka_emanuelconrado/manager_node.hpp"

namespace manager_node_cpp
{
ManagerNode::ManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("manager_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.state_machine", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.qty_points", rclcpp::ParameterValue(4));
  declare_parameter("waypoints.points", rclcpp::ParameterValue(std::vector<double>{}));
}

ManagerNode::~ManagerNode() {
}

CallbackReturn ManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}


CallbackReturn ManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");
  
  pub_goto_->on_activate(); 
  pub_goto_relative_->on_activate(); 
  pub_have_goal_->on_activate(); 

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");
  
  pub_goto_->on_deactivate();
  pub_goto_relative_->on_deactivate();
  pub_have_goal_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}

void ManagerNode::getParameters() {
  get_parameter("rate.state_machine", _rate_state_machine_);
  get_parameter("waypoints.qty_points", _waypoints_qty_points_);
  get_parameter("waypoints.points", _waypoints_points_);

  std::cout <<  _waypoints_points_[3] << std::endl;
}

void ManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");
  
  pub_goto_ = create_publisher<geometry_msgs::msg::Pose>("/uav1/goto", 1);
  pub_goto_relative_ = create_publisher<geometry_msgs::msg::Pose>("uav1/goto_ralative", 1);

  sub_have_goal_ = create_subscription<std_msgs::msg::Bool>("/uva1/have_goal", 1, std::bind(&ManagerNode::subOpHaveGoal, this, std::placeholders::_1));
  pub_have_goal_ = create_publisher<std_msgs::msg::Bool>("uav1/have_goal", 1);

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("/uva1/odometry", 1, std::bind(&ManagerNode::subOpOdometry, this, std::placeholders::_1));
  
}

void ManagerNode::subOpOdometry(nav_msgs::msg::Odometry odometry){
  
}

void ManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_op_goto_ = create_wall_timer(std::chrono::duration<double>(1.0/ _rate_state_machine_), std::bind(&ManagerNode::tmrOpGoto, this), nullptr);

}

void ManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  clt_takeoff_ = create_client<std_srvs::srv::Trigger>("/uav1/takeoff");
  clt_land_ = create_client<std_srvs::srv::Trigger>("/uav1/land"); 
  clt_arm_ = create_client<std_srvs::srv::Trigger>("/uav1/arm"); 
  clt_disarm_ = create_client<std_srvs::srv::Trigger>("/uav1/disarm");

}

void ManagerNode::subOpHaveGoal(std_msgs::msg::Bool have_goal){
  have_goal_ = have_goal.data;
}

void ManagerNode::tmrOpGoto(){
  pub_goto_->publish(pose_);
}
}  // namespace manager_node_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manager_node_cpp::ManagerNode)
