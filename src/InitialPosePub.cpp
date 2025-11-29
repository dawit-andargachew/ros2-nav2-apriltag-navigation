/**
 * Node: initial_pose_pub
 *
 * Purpose:
 *   - Publishes an initial pose for the robot at the origin (0,0,0) in the "map" frame.
 *   - Keeps republishing until AMCL acknowledges the pose, ensuring the localization starts correctly.
 *
 * Inputs:
 *   - "amcl_pose" (geometry_msgs::msg::PoseWithCovarianceStamped): feedback from AMCL to confirm pose acceptance.
 *
 * Outputs:
 *   - "initialpose" (geometry_msgs::msg::PoseWithCovarianceStamped): initial pose sent to AMCL for localization.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#define QUEUE_SIZE 10
#define TIMER_PERIOD_SEC 1
#define PUBLISH_RATE_HZ 1

class InitialPosePub : public rclcpp::Node
{
public:
    InitialPosePub()
    : Node("initial_pose_pub")
    {
        // Publisher for the initial pose
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", QUEUE_SIZE);

        // Subscriber to check when AMCL accepts the initial pose 
        // (so we don't lose it before the whole initialitazion is up)
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", QUEUE_SIZE,
            std::bind(&InitialPosePub::amclCallback, this, std::placeholders::_1));

        // Timer to republish intial pose until confirmation
        timer_ = this->create_wall_timer(
            std::chrono::seconds(TIMER_PERIOD_SEC),
            std::bind(&InitialPosePub::publishInitialPose, this));

        RCLCPP_INFO(this->get_logger(), "InitialPosePub started");
    }

private:
    // Keeps publishing initial pose until AMCL responds
    void publishInitialPose()
    {
        if (initialized_) return;

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
    }

    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
    {
        // returns immediately so it doesn't print the next message every time but just one
        if (initialized_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "AMCL pose received - stopping initial pose publishing");
        initialized_ = true;
        timer_->cancel();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool initialized_ = false; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePub>());
    rclcpp::shutdown();
    return 0;
}
