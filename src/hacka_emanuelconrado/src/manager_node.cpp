#include "hacka_emanuelconrado/manager_node.hpp"

namespace manager_node_cpp
{
ManagerNode::ManagerNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("manager_node", "", options) {
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.state_machine", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.qty_points", rclcpp::ParameterValue(1));
  declare_parameter<std::vector<double>>("waypoints.points", std::vector<double>{1.0,1.0,1.0});

}

ManagerNode::~ManagerNode() {
}

CallbackReturn ManagerNode::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();
  configClients();

  return CallbackReturn::SUCCESS;
}


CallbackReturn ManagerNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Activating");

  pub_have_goal_->on_activate();
  pub_goto_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagerNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_have_goal_->on_deactivate();
  pub_goto_->on_deactivate();

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
  RCLCPP_INFO(get_logger(), "Loading parameters");

  get_parameter("rate.state_machine", _rate_state_machine_);
  get_parameter("waypoints.qty_points", _waypoints_qty_points_);
  get_parameter("waypoints.points", _waypoints_points_);
  
}

void ManagerNode::configPubSub() {
  RCLCPP_INFO(get_logger(), "initPubSub");
  
  sub_have_goal_ = create_subscription<std_msgs::msg::Bool>("/uav1/have_goal", 1, std::bind(&ManagerNode::subOpHaveGoal, this, std::placeholders::_1));

  pub_goto_ = create_publisher<geometry_msgs::msg::Pose>("uav1/goto", 1);

  pub_have_goal_ = create_publisher<std_msgs::msg::Bool>("uav1/have_goal", 1);
  
}

void ManagerNode::configTimers() {
  RCLCPP_INFO(get_logger(), "initTimers");

}

void ManagerNode::configClients(){
  RCLCPP_INFO(get_logger(), "initClients");

  clt_arm_ = create_client<std_srvs::srv::Trigger>("/uav1/arm");
  clt_land_ = create_client<std_srvs::srv::Trigger>("uav1/land");
  clt_takeoff_ = create_client<std_srvs::srv::Trigger>("uav1/takeoff");
}

void ManagerNode::configServices() {
  RCLCPP_INFO(get_logger(), "initServices");

  srv_start_state_machine_ = this->create_service<std_srvs::srv::Trigger>("start_state_machine", std::bind(&ManagerNode::stateTriggerRequest, this, std::placeholders::_1, std::placeholders::_2));

}

void ManagerNode::stateTriggerRequest([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
  response->success = true;
  response->message = "State Machine started";

  if(have_goal_){
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void{
        RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
    };

    clt_arm_->async_send_request(request, callback_result);
  
    seconds timer_(5);
    auto inicio = steady_clock::now();
    
    while(duration_cast<seconds>(steady_clock::now() - inicio) < timer_){
        if(!have_goal_){
          takeoff();
          return;
        }
    }


    RCLCPP_INFO(get_logger(), "Drone pousando");
    clt_land_->async_send_request(request, callback_result);

  }else RCLCPP_INFO(get_logger(), "Drone não ativado");

  
}

void ManagerNode::takeoff(){
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void{
      RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
  };

  clt_takeoff_->async_send_request(request,callback_result);

  seconds timer_(5);
  auto inicio = steady_clock::now();

  while (duration_cast<seconds>(steady_clock::now() - inicio) < timer_){
          if(!have_goal_){
             goingTo();
          }
  }
}

void ManagerNode::goingTo(){
  while(true){
    getNextPose();
    pub_goto_->publish(goto_pos_);

    seconds timer_(5);
    auto inicio = steady_clock::now();

    while(duration_cast<seconds>(steady_clock::now() - inicio) < timer_){
      if(!have_goal_){
        break;
      }
    }

    if(have_goal_){
      break;
    }

    if(_waypoints_qty_points_ == 0){
      break;
    }

  }
}

void ManagerNode::subOpHaveGoal(std_msgs::msg::Bool have_goal){
  have_goal_ = have_goal.data;
}

void ManagerNode::getNextPose(){
  int start_point = _waypoints_points_.size() - (_waypoints_qty_points_ * 3);
  goto_pos_.position.x = _waypoints_points_[start_point];
  goto_pos_.position.y = _waypoints_points_[start_point + 1];
  goto_pos_.position.z = _waypoints_points_[start_point + 2];

  _waypoints_qty_points_--;
}

}  // namespace manager_node_cpp

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manager_node_cpp::ManagerNode)
