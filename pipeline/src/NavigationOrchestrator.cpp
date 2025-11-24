#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

class NavigationOrchestrator : public rclcpp::Node
{
public:
  NavigationOrchestrator() : Node("navigation_orchestrator")
  {
    // Subscribing to GoalDetector publisher to get the goal position
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",   
        rclcpp::QoS(10),
        std::bind(&NavigationOrchestrator::goalCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "NavigationOrchestrator is ready and listening for goal_pose.");

  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: frame_id='%s' pos(%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationOrchestrator>());
  rclcpp::shutdown();
  return 0;
}