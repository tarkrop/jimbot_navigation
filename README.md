## 빌드
<br/>

```
cd ros2_ws
colcon build --symlink-install --packages-select jimbot_navigation
```

<br/>
<br/>

## Planner 기능을 사용하는 경우
<br/>

```
ros2 launch jimbot_navigation navigation_launch.py
```

<br/>
<br/>

## Planner에게 Goal 전달하기

<br/>
Action 타입: nav2_msgs::action::NavigateToPose (#include "nav2_msgs/action/navigate_to_pose.hpp")
<br/>
아래와 같은 구조의 함수를 작성해서 클래스 선언 시 세팅할 수 있다.
<br/>
<br/>

```
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

void send_compute_path_goal(const geometry_msgs::msg::PoseStamped & goal_pose) // 타입은 재량
{
  if (!this->action_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Planning server not available after waiting");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  // Goal 설정...

  RCLCPP_INFO(this->get_logger(), "Sending goal to server...");

  // Action Client 함수 정의
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // Response를 받을 시 수행되는 함수, 아래는 에러 발생 여부 확인
  send_goal_options.goal_response_callback =
    [this](GoalHandleNavigateToPose::SharedPtr future) {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal was accepted by server. Waiting for result...");
      }
    };

  // 피드백을 받는 경우 실행, 피드백 보내지 않도록 세팅해놓아서 아래 코드 그대로 사용
  send_goal_options.feedback_callback =
    [this](GoalHandleNavigateToPose::SharedPtr goal_handle,
      const std::shared_ptr<const ComputePathToPose::Feedback> feedback) {
      (void)goal_handle;
      (void)feedback;
      RCLCPP_INFO(this->get_logger(), "Received feedback (no specific data in ComputePathToPose feedback)");
    };

// case 별로 성공 여부 확인, result.code = rclcpp_action::ResultCode::SUCCEEDED 일 때
// 다음 행동으로 넘어가도록 플래그를 바꾸거나 함수를 실행하도록 할 수 있음
send_goal_options.result_callback =
  [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
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
  
  // Action Server에 목표를 보내는 코드
  this->action_path_client_->async_send_goal(goal_msg, send_goal_options);
}
```
