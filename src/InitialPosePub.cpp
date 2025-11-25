/* 
pipeline/src/InitialPosePub.cpp

Input: 
-
Output:
-

*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#define PUBLISHER_QUEUE_SIZE 10
#define TIMER_PERIOD_SEC 1

class InitialPosePub : public rclcpp::Node
{
public:
    InitialPosePub()
    : Node("initial_pose_pub")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", PUBLISHER_QUEUE_SIZE);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(TIMER_PERIOD_SEC),
            std::bind(&InitialPosePub::publishInitialPose, this));

        RCLCPP_INFO(this->get_logger(), "InitialPosePub started");
    }

private:
    void publishInitialPose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        msg.header.stamp = now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        msg.pose.pose.orientation.w = 1.0;

        // Basic covariance 'cause AMCL requires non-zero covariance
        msg.pose.covariance[0] = 0.5;
        msg.pose.covariance[7] = 0.5;
        msg.pose.covariance[35] = 0.1;

        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published initial pose (0,0,0)");

        // Publish only once
        timer_->cancel();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePub>());
    rclcpp::shutdown();
    return 0;
}
