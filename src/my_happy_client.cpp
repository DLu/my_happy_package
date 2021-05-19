#include <rclcpp/rclcpp.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace my_happy_package
{
trajectory_msgs::msg::JointTrajectoryPoint makePoint(double t, double position,
                                                     double velocity = 0.0, double acceleration = 0.0)
{
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions.push_back(position);
  pt.velocities.push_back(velocity);
  pt.accelerations.push_back(acceleration);
  pt.time_from_start = rclcpp::Duration::from_seconds(t);
  return pt;
}

class TrajectoryActionClient : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit TrajectoryActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_action_client", options)
  {
    client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(this,
                                                                      "/trajectory");
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&TrajectoryActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names.push_back("wrist_extension");
    goal.trajectory.points.push_back(makePoint(0.0, base_pos_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 0.5, base_pos_ + arm_swing_));
    goal.trajectory.points.push_back(makePoint(swing_duration_ * 1.5, base_pos_ - arm_swing_));

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TrajectoryActionClient::goal_response_callback, this, _1);
    send_goal_options.result_callback = std::bind(&TrajectoryActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  double arm_swing_ {0.1};
  double swing_duration_ {10.0};
  double base_pos_ {0.1};

  void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), result.result->error_string.c_str());
  }
};  // class TrajectoryActionClient

}  // namespace my_happy_package

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_happy_package::TrajectoryActionClient>());
  rclcpp::shutdown();
  return 0;
}
