#include <rclcpp/rclcpp.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace my_happy_package
{
class TrajectoryActionServer : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit TrajectoryActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "trajectory",
      std::bind(&TrajectoryActionServer::handle_goal, this, _1, _2),
      std::bind(&TrajectoryActionServer::handle_cancel, this, _1),
      std::bind(&TrajectoryActionServer::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Server Ready");
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with %zu joints", goal->trajectory.joint_names.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TrajectoryActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
   }
};  // class TrajectoryActionServer

}  // namespace my_happy_package

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_happy_package::TrajectoryActionServer>());
  rclcpp::shutdown();
  return 0;
}
