#ifndef MANAGER_NODE_CPP__MANAGER_NODE_HPP
#define MANAGER_NODE_CPP__MANAGER_NODE_HPP

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace manager_node_cpp
{
class ManagerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~ManagerNode() override;

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

  void getParameters();
  void configPubSub();
  void configTimers();
  void configServices();

//Clients-------

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_takeoff_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_land_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_arm_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_disarm_;

//Publishers

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_goto_relative_; 
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr pub_have_goal_;

//Subscribers

  rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr sub_have_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odometry_;

  
//Timers

  rclcpp::TimerBase::SharedPtr tmr_op_goto_;
  
//servs


//Funcs

  void tmrOpGoto();
  void subOpHaveGoal(std_msgs::msg::Bool have_goal);
  void subOpOdometry(nav_msgs::msg::Odometry odometry);

//Variables

  bool have_goal_;
  geometry_msgs::msg::Pose pose_;
  double _rate_state_machine_;
  int _waypoints_qty_points_;
  std::vector<double> _waypoints_points_;
};
}

#endif
