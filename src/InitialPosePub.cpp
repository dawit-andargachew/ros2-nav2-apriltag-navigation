/**
 * Node: initial_pose_pub
 *
 * Purpose:
 *   - Publishes an initial pose for the robot. The Initial Pose is configurable in config/InitialPose.yaml. 
 *   - The default is at the origin (0,0,0) in the "map" frame.
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
        // Parameters loading from config yaml file
        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.0);
        this->declare_parameter<double>("qw", 1.0);
        this->declare_parameter<double>("cov_xx", 0.5);
        this->declare_parameter<double>("cov_yy", 0.5);
        this->declare_parameter<double>("cov_aa", 0.1);
        loadParameters();

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
    void loadParameters()
    {
        x_ = this->get_parameter("x").as_double();
        y_ = this->get_parameter("y").as_double();
        z_ = this->get_parameter("z").as_double();

        qw_ = this->get_parameter("qw").as_double();

        cov_xx_ = this->get_parameter("cov_xx").as_double();
        cov_yy_ = this->get_parameter("cov_yy").as_double();
        cov_aa_ = this->get_parameter("cov_aa").as_double();
    }

    // Keeps publishing initial pose until AMCL responds
    void publishInitialPose()
    {
        if (initialized_) return;

        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        msg.header.stamp = now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = x_;
        msg.pose.pose.position.y = y_;
        msg.pose.pose.position.z = z_;

        msg.pose.pose.orientation.w = qw_;

        // Basic covariance 'cause AMCL requires non-zero covariance
        msg.pose.covariance[0] = cov_xx_;
        msg.pose.covariance[7] = cov_yy_;
        msg.pose.covariance[35] = cov_aa_;

        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published initial pose (%.1f, %.1f, %.1f)", x_, y_, z_);
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
    double x_, y_, z_;
    double qw_;
    double cov_xx_, cov_yy_, cov_aa_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePub>());
    rclcpp::shutdown();
    return 0;
}
