/* 
pipeline/src/NavigationOrchestrator.cpp

Input: 
-
Output:
-

*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "pipeline/srv/detect_tables.hpp"

using namespace std::chrono_literals;
using DetectTables = pipeline::srv::DetectTables;

#define SUBSCRIBER_QUEUE_SIZE 10
#define FEEDBACK_THROTTLE_TIMER_SEC 0.5
#define SERVICE_WAIT_TIMER_SEC 2

class NavigationOrchestrator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationOrchestrator() : Node("navigation_orchestrator")
    {
        // Creates navigation action client
        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, 
            "navigate_to_pose"
        );

        // Subscribing to GoalDetector publisher to get the goal position
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose",   
            SUBSCRIBER_QUEUE_SIZE,
            std::bind(&NavigationOrchestrator::goalCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "NavigationOrchestrator is ready and listening for goal_pose.");

        // Creates table detector client
        table_detector_client_ = this->create_client<DetectTables>("detect_tables");
    
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Client<DetectTables>::SharedPtr table_detector_client_;
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Collects the goal position
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: frame_id='%s' pos(%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        if (!navigate_to_pose_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(),
                         "NavigateToPose action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *msg;

        RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.feedback_callback =
            std::bind(&NavigationOrchestrator::feedbackCallback, this,
                      std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&NavigationOrchestrator::resultCallback, this,
                      std::placeholders::_1);

        // Navigates to the goal position
        navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedbackCallback(GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        auto now = this->now();

        // Throttle feedback to once every half seconds so it doesn't update too frequently
        if ((now - last_feedback_time_).seconds() < FEEDBACK_THROTTLE_TIMER_SEC) {
            return;
        }
        last_feedback_time_ = now;

        RCLCPP_INFO(this->get_logger(),
                    "Nav2 Feedback: distance remaining = %.2f",
                    feedback->distance_remaining);
    }

    void resultCallback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
                callTableDetectionServer();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Navigation error");
                break;
        }
    }

    void callTableDetectionServer()
    {
        if (!table_detector_client_->wait_for_service(SERVICE_WAIT_TIMER_SEC)) {
            RCLCPP_ERROR(this->get_logger(),
                        "TableDetectionServer not available");
            return;
        }

        auto request = std::make_shared<DetectTables::Request>();

        RCLCPP_INFO(this->get_logger(), "Calling TableDetectionServer");

        table_detector_client_->async_send_request(
            request,
            [this](rclcpp::Client<DetectTables>::SharedFuture future)
            {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(),
                                "TableDetectionServer finished successfully.");
                }
                catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(),
                                "Service call failed: %s", e.what());
                }
            }
        );
    }


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationOrchestrator>());
    rclcpp::shutdown();
    return 0;
}