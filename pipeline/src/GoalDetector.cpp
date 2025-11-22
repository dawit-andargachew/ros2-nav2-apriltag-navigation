/* pipeline/src/GoalDetector.cpp

Input: 
-  from looking at the apriltag_ros we see it publishes the tf's for the tags it detects
-  we can use tf2 to lookup the transforms of the two tags in the map frame
-  then compute the midpoint between the two tags
-  and publish that as the goal pose
-Apriltag IDs: 1 and 2 of family tag36h11
- TF's: camera_frame -> base_link -> odom -> map
Output:
    - publish to /goal_pose as geometry_msgs/PoseStamped
        - the midpoint between the two apriltags, and with what orientation? TBD
*/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#define PUBLISHER_QUEUE_SIZE 10
#define TIMER_PERIOD_MS 1000
#define THROTTLE_LOG_MS 1000
#define FIND_TF_TIMEOUT_MS 1000
class GoalDetector : public rclcpp::Node {
public:
    GoalDetector() : Node("goal_detector") {
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

    void computeGoalCallback() {
        try {
            // Look up transforms for both tags in map frame, i hope this exists straight to map, otherwise
            // we will chain from camera to base_link to odom to map
            geometry_msgs::msg::TransformStamped tag1_tf = tf_buffer_->lookupTransform(
                "map", "tag36h11:1", tf2::TimePointZero, std::chrono::milliseconds(FIND_TF_TIMEOUT_MS));
            
            geometry_msgs::msg::TransformStamped tag2_tf = tf_buffer_->lookupTransform(
                "map", "tag36h11:10", tf2::TimePointZero, std::chrono::milliseconds(FIND_TF_TIMEOUT_MS));

            // midpoint
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            
            goal_pose.pose.position.x = (tag1_tf.transform.translation.x + tag2_tf.transform.translation.x) / 2.0;
            goal_pose.pose.position.y = (tag1_tf.transform.translation.y + tag2_tf.transform.translation.y) / 2.0;
            goal_pose.pose.position.z = (tag1_tf.transform.translation.z + tag2_tf.transform.translation.z) / 2.0;
            
            // for now, use tag1's orientation
            //TODO: what way should the robot face?
            goal_pose.pose.orientation = tag1_tf.transform.rotation;

            goal_pub_->publish(goal_pose);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_LOG_MS,
                "Goal at: (%.2f, %.2f, %.2f)", 
                goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);

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