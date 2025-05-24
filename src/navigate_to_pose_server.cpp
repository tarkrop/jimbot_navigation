#include "jimbot_navigation/navigate_to_pose_server.hpp"

// 생성자 구현
NavigateToPoseServer::NavigateToPoseServer(const rclcpp::NodeOptions & options)
: Node("navigate_to_pose_server", options)
{
  current_plan_control_goal_handle_ = nullptr;

  this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose",
    std::bind(&NavigateToPoseServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigateToPoseServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavigateToPoseServer::handle_accepted, this, std::placeholders::_1)
  );

  this->plan_control_server_ = rclcpp_action::create_server<PlanControl>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "plan_control",
    std::bind(&NavigateToPoseServer::plan_control_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigateToPoseServer::plan_control_cancel, this, std::placeholders::_1),
    std::bind(&NavigateToPoseServer::plan_control_accepted, this, std::placeholders::_1)
  );

  this->action_path_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "compute_path_to_pose");

    this->action_follow_client_ = rclcpp_action::create_client<FollowPath>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "follow_path");

  current_pose_callback();

  RCLCPP_INFO(this->get_logger(), "NavigateToPose server has been started.");
}

// NavigateToPose 서버

rclcpp_action::GoalResponse NavigateToPoseServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with pose: (%f, %f, %f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z);

  (void)uuid;
  
  if(!cur_pose_initialized_){
    RCLCPP_INFO(this->get_logger(), "Current Pose is Not Initialized!!!!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToPoseServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  if(goal_handle->is_executing()){
    return rclcpp_action::CancelResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseServer::handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  std::thread{std::bind(&NavigateToPoseServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void NavigateToPoseServer::execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto feedback = std::make_shared<NavigateToPose::Feedback>();
  auto goal_pose = goal_handle->get_goal()->pose;
  auto result = std::make_shared<NavigateToPose::Result>();

  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Get Goal From RVIZ");
  }

  send_compute_path_goal(goal_pose);
}

// PlanControl 서버

rclcpp_action::GoalResponse NavigateToPoseServer::plan_control_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PlanControl::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with pose: (%f, %f, %f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y, goal->pose.pose.position.z);

  (void)uuid;
  
  if(!cur_pose_initialized_){
    RCLCPP_INFO(this->get_logger(), "Current Pose is Not Initialized!!!!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse NavigateToPoseServer::plan_control_cancel(
  const std::shared_ptr<GoalHandlePlanControl> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseServer::plan_control_accepted(const std::shared_ptr<GoalHandlePlanControl> goal_handle)
{
  // if (current_plan_control_goal_handle_ != nullptr)
  // {
  //   auto result = std::make_shared<PlanControl::Result>();
  //   current_plan_control_goal_handle_->abort(result);
  //   RCLCPP_INFO(this->get_logger(), "Aborting previous goal");
  // }

  current_plan_control_goal_handle_ = goal_handle;
  current_plan_control_goal_handle_->execute();
  std::thread{std::bind(&NavigateToPoseServer::plan_control_excute, this, std::placeholders::_1), current_plan_control_goal_handle_}.detach();
}

void NavigateToPoseServer::plan_control_excute(const std::shared_ptr<GoalHandlePlanControl> goal_handle)
{
  is_plan_completed_ = false;
  is_mpc_completed_ = false;

  RCLCPP_INFO(this->get_logger(), "==================================");
  RCLCPP_INFO(this->get_logger(), "Executing goal...");

  auto goal_pose = goal_handle->get_goal()->pose;
  if (!this->action_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Planning server not available after waiting");
    return;
  }

  auto goal_msg = ComputePathToPose::Goal();

  goal_msg.goal.header.stamp.sec = 0;
  goal_msg.goal.header.stamp.nanosec = 0;
  goal_msg.goal.header.frame_id = "map";
  goal_msg.goal.pose = goal_pose.pose;
  goal_msg.planner_id = "GridBased";
  goal_msg.use_start = true;
  goal_msg.start = cur_pos_;

  RCLCPP_INFO(this->get_logger(), "Sending goal to server...");


  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](GoalHandleComputePathToPose::SharedPtr future) {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by server. Waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleComputePathToPose::SharedPtr goal_handle,
      const std::shared_ptr<const ComputePathToPose::Feedback> feedback) {
      (void)goal_handle;
      (void)feedback;
      RCLCPP_INFO(this->get_logger(), "Received feedback (no specific data in ComputePathToPose feedback)");
    };

send_goal_options.result_callback =
  [this](const GoalHandleComputePathToPose::WrappedResult & result) {
    switch (result.code) {
      
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Compute Path Complete");
        send_followpath_pc(result.result->path);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  };

  this->action_path_client_->async_send_goal(goal_msg, send_goal_options);
}


// ComputePath 클라이언트

void NavigateToPoseServer::send_compute_path_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!this->action_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Planning server not available after waiting");
    return;
  }

  auto goal_msg = ComputePathToPose::Goal();

  goal_msg.goal.header.stamp.sec = 0;
  goal_msg.goal.header.stamp.nanosec = 0;
  goal_msg.goal.header.frame_id = "map";
  goal_msg.goal.pose = goal_pose.pose;
  goal_msg.planner_id = "GridBased";
  goal_msg.use_start = true;
  goal_msg.start = cur_pos_;

  RCLCPP_INFO(this->get_logger(), "Sending goal to server...");


  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](GoalHandleComputePathToPose::SharedPtr future) {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by server. Waiting for result...");
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleComputePathToPose::SharedPtr goal_handle,
      const std::shared_ptr<const ComputePathToPose::Feedback> feedback) {
      (void)goal_handle;
      (void)feedback;
      RCLCPP_INFO(this->get_logger(), "Received feedback (no specific data in ComputePathToPose feedback)");
    };

send_goal_options.result_callback =
  [this](const GoalHandleComputePathToPose::WrappedResult & result) {
    switch (result.code) {
      
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Compute Path Complete");
        // planned_path_ = result.result->path;
        send_followpath(result.result->path);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  };

  this->action_path_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigateToPoseServer::current_pose_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cur_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "mcl_pose",
    qos,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
    { 
      cur_pos_.header.stamp.sec = 0;
      cur_pos_.header.stamp.nanosec = 0;
      cur_pos_.header.frame_id = "map";
      cur_pos_.pose = msg->pose.pose;

      if(!cur_pose_initialized_){
        cur_pose_initialized_ = true;
      }
    }
  );
}

// FollowPath 클라이언트

void NavigateToPoseServer::send_followpath(const nav_msgs::msg::Path & plan_path)
{
  RCLCPP_INFO(this->get_logger(), "============================================================");
  // if (is_following_) {
  //   RCLCPP_INFO(this->get_logger(), "New path available. Canceling current FollowPath action...");
  //   action_follow_client_->async_cancel_goal(current_follow_goal_handle_);
  //   current_follow_goal_handle_ = nullptr;
  // }
  if (!this->action_follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Follow server not available after waiting");
    return;
  }  

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = plan_path;
  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
  [this](GoalHandleFollowPath::SharedPtr goal_handle) {
    if (!goal_handle.get()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else 
    {
      // current_follow_goal_handle_ = goal_handle;
      // is_following_ = true;
      RCLCPP_INFO(this->get_logger(), "Goal was accepted by server. Waiting for result...");
    }
  };

  send_goal_options.feedback_callback =
    [this](GoalHandleFollowPath::SharedPtr goal_handle,
      const std::shared_ptr<const GoalHandleFollowPath::Feedback> feedback) {
      (void)goal_handle;
      (void)feedback;
      // goal_dis_ = feedback->distance_to_goal;
      // RCLCPP_INFO(this->get_logger(), "Goal Distance: %.2f", feedback->distance_to_goal);

    };
    
    send_goal_options.result_callback = 
      [this](const GoalHandleFollowPath::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
      }
    };

  action_follow_client_->async_send_goal(goal_msg, send_goal_options);
}

void NavigateToPoseServer::send_followpath_pc(const nav_msgs::msg::Path & plan_path)
{
  RCLCPP_INFO(this->get_logger(), "============================================================");
  // if (is_following_) {
  //   RCLCPP_INFO(this->get_logger(), "New path available. Canceling current FollowPath action...");
  //   action_follow_client_->async_cancel_goal(current_follow_goal_handle_);
  //   current_follow_goal_handle_ = nullptr;
  // }
  if (!this->action_follow_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Follow server not available after waiting");
    return;
  }

  

  auto goal_msg = FollowPath::Goal();
  goal_msg.path = plan_path;
  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

  send_goal_options.goal_response_callback =
  [this](GoalHandleFollowPath::SharedPtr goal_handle) {
    if (!goal_handle.get()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else 
    {
      // current_follow_goal_handle_ = goal_handle;
      // is_following_ = true;
      RCLCPP_INFO(this->get_logger(), "Goal was accepted by server. Waiting for result...");
    }
  };

  send_goal_options.feedback_callback =
    [this](GoalHandleFollowPath::SharedPtr goal_handle,
      const std::shared_ptr<const GoalHandleFollowPath::Feedback> feedback) {
      (void)goal_handle;
      auto feedback_pc = std::make_shared<PlanControl::Feedback>();
      feedback_pc->distance_to_goal = feedback->distance_to_goal;
      current_plan_control_goal_handle_->publish_feedback(feedback_pc);
      RCLCPP_INFO(this->get_logger(), "Goal Distance: %.2f", feedback_pc->distance_to_goal);

    };
    
    send_goal_options.result_callback = 
      [this](const GoalHandleFollowPath::WrappedResult & result) {
        auto result_pc = std::make_shared<PlanControl::Result>();
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was succeeded Final");
            result_pc->success = true;
            current_plan_control_goal_handle_->succeed(result_pc);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            result_pc->success = false;
            current_plan_control_goal_handle_->abort(result_pc);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            result_pc->success = false;
            current_plan_control_goal_handle_->abort(result_pc);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            result_pc->success = false;
            current_plan_control_goal_handle_->abort(result_pc);
            break;
      }
    };

  action_follow_client_->async_send_goal(goal_msg, send_goal_options);
}

// main 함수
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server_node = std::make_shared<NavigateToPoseServer>();
  rclcpp::spin(action_server_node);

  rclcpp::shutdown();

  return 0;
}
