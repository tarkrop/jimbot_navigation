#ifndef NAVIGATE_TO_POSE_SERVER__NAVIGATE_TO_POSE_SERVER_HPP_
#define NAVIGATE_TO_POSE_SERVER__NAVIGATE_TO_POSE_SERVER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "jimbot_msgs/action/plan_control.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


class NavigateToPoseServer : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using PlanControl = jimbot_msgs::action::PlanControl;
  using GoalHandlePlanControl = rclcpp_action::ServerGoalHandle<PlanControl>;

  explicit NavigateToPoseServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void send_compute_path_goal(const geometry_msgs::msg::PoseStamped & goal_pose);
  void send_followpath(const nav_msgs::msg::Path & plan_path);
  void send_followpath_pc(const nav_msgs::msg::Path & plan_path);
private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  rclcpp_action::GoalResponse plan_control_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanControl::Goal> goal);
  rclcpp_action::CancelResponse plan_control_cancel(
    const std::shared_ptr<GoalHandlePlanControl> goal_handle);
  void plan_control_accepted(const std::shared_ptr<GoalHandlePlanControl> goal_handle);
  void plan_control_excute(const std::shared_ptr<GoalHandlePlanControl> goal_handle);

  void goal_response_callback(std::shared_future<GoalHandleComputePathToPose::SharedPtr> future);

  // 서버로부터 피드백을 받았을 때 호출
  void feedback_callback(
    GoalHandleComputePathToPose::SharedPtr goal_handle, // 현재 목표 핸들
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback); // 수신된 피드백 데이터

  // 서버로부터 최종 결과를 받았을 때 호출
  void result_callback(const GoalHandleComputePathToPose::WrappedResult & result);


  void current_pose_callback();


  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp_action::Server<PlanControl>::SharedPtr plan_control_server_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_path_client_;

  rclcpp_action::Client<FollowPath>::SharedPtr action_follow_client_;
  // GoalHandleFollowPath::SharedPtr current_follow_goal_handle_;
  std::shared_ptr<GoalHandlePlanControl> current_plan_control_goal_handle_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cur_pos_sub_;
  
  geometry_msgs::msg::PoseStamped cur_pos_;
  float goal_dis_ = 0.0;
  // nav_msgs::msg::Path planned_path_;
  
  bool cur_pose_initialized_ = false;
  bool path_updated_ = false;
  bool is_following_ = false;

  bool is_plan_completed_ = false;
  bool is_mpc_completed_ = false;
};

#endif // CUSTOM_RVIZ_SERVER__NAVIGATE_TO_POSE_SERVER_HPP_