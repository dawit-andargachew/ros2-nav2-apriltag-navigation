/*
 * Node: goal_detector
 *
 * Purpose:
 *   - Look up two AprilTag frames in the TF tree.
 *   - Compute a goal pose between them in the <output_frame> frame.
 *   - Orient the robot perpendicular to the line joining the tags.
 *
 * Inputs:
 *   - TF frames: <output_frame> -> <tag1_frame>, <tag2_frame>
 *
 * Parameters:
 *   - output_frame (string, default: "map")
 *   - tag1_frame (string, default: "tag36h11:1")
 *   - tag2_frame (string, default: "tag36h11:10")
 *   - yaw_offset_rad (double, default: -pi/2)
 *   - distance_offset (double, default: 0.0)
 *
 * Outputs:
 *   - Topic: /goal_pose (geometry_msgs/PoseStamped, frame: output_frame)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr int PUBLISHER_QUEUE_SIZE = 10;
constexpr int TIMER_PERIOD_MS      = 1000;
constexpr int THROTTLE_LOG_MS      = 1000;
constexpr int FIND_TF_TIMEOUT_MS   = 1000;
}
class GoalDetector : public rclcpp::Node {
public:
    GoalDetector() : Node("goal_detector") {
        output_frame_   = this->declare_parameter<std::string>("output_frame", "map");
        tag1_frame_  = this->declare_parameter<std::string>("tag1_frame", "tag36h11:1");
        tag2_frame_  = this->declare_parameter<std::string>("tag2_frame", "tag36h11:10");
        yaw_offset_  = this->declare_parameter<double>("yaw_offset_rad", (-1*M_PI) / 2.0);
        distance_offset_ = this->declare_parameter<double>("distance_offset", 0.0); // metres away from midpoint

        RCLCPP_INFO(this->get_logger(), "GoalDetector parameters: output_frame=%s tag1=%s tag2=%s yaw_offset=%.3f dist_offset=%.3f",
                    output_frame_.c_str(), tag1_frame_.c_str(), tag2_frame_.c_str(), yaw_offset_, distance_offset_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", PUBLISHER_QUEUE_SIZE);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(TIMER_PERIOD_MS),
            std::bind(&GoalDetector::computeGoalCallback, this));
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string output_frame_;
    std::string tag1_frame_;
    std::string tag2_frame_;
    double yaw_offset_;
    double distance_offset_;
    void computeGoalCallback() {
        try {
            //check both tags exist and check transform lookup
            if (!tf_buffer_->canTransform(output_frame_, tag1_frame_, tf2::TimePointZero,
                                          std::chrono::milliseconds(FIND_TF_TIMEOUT_MS)) ||
                !tf_buffer_->canTransform(output_frame_, tag2_frame_, tf2::TimePointZero,
                                          std::chrono::milliseconds(FIND_TF_TIMEOUT_MS))) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_LOG_MS,
                    "One or both tag transforms not available yet");
                return;
            }

            // Look up transforms for both tags in the output_frame_. TF will chain through intermediate frames if needed.
            geometry_msgs::msg::TransformStamped tag1_tf = tf_buffer_->lookupTransform(
                output_frame_, tag1_frame_, tf2::TimePointZero, std::chrono::milliseconds(FIND_TF_TIMEOUT_MS));
            
            geometry_msgs::msg::TransformStamped tag2_tf = tf_buffer_->lookupTransform(
                output_frame_, tag2_frame_, tf2::TimePointZero, std::chrono::milliseconds(FIND_TF_TIMEOUT_MS));

            // midpoint
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = output_frame_;
            goal_pose.header.stamp = tag1_tf.header.stamp;
            
            double x1 = tag1_tf.transform.translation.x;
            double y1 = tag1_tf.transform.translation.y;
            double x2 = tag2_tf.transform.translation.x;
            double y2 = tag2_tf.transform.translation.y;

            goal_pose.pose.position.x = (x1 + x2) / 2.0;
            goal_pose.pose.position.y = (y1 + y2) / 2.0;
            goal_pose.pose.position.z = (tag1_tf.transform.translation.z + tag2_tf.transform.translation.z) / 2.0;

            
            //facing perpendicular to the line between the two tags
            double dx = x2 - x1;
            double dy = y2 - y1;
            // base edge yaw
            double base_yaw = std::atan2(dy, dx);
            // desired yaw is the edge yaw plus configured offset (default = pi/2 to face perpendicular)
            double yaw = base_yaw + yaw_offset_;

            // apply distance offset along the perpendicular direction
            // perpendicular unit vector to (dx,dy) is (-dy, dx) normalized
            double edge_len = std::hypot(dx, dy);
            if (edge_len > 1e-6 && std::abs(distance_offset_) > 1e-9) {
                double ux = -dy / edge_len;
                double uy = dx / edge_len;
                goal_pose.pose.position.x += ux * distance_offset_;
                goal_pose.pose.position.y += uy * distance_offset_;
            }

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            goal_pose.pose.orientation = tf2::toMsg(q);
    
            goal_pub_->publish(goal_pose);
            
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), THROTTLE_LOG_MS,
                "Goal at: (%.2f, %.2f, %.2f) in %s",
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                output_frame_.c_str()
            );

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_LOG_MS,
                "Could not get tag transforms: %s", ex.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalDetector>());
    rclcpp::shutdown();
    return 0;
}