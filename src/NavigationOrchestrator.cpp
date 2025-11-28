/*
NavigationOrchestrator Node

- Orchestrates robot navigation using Nav2 and table detection service.
- Implements a simple state machine: waits for goal, sends navigation action, handles feedback/result, and triggers table detection.
- Subscribes to goal poses, sends goals to Nav2, and calls table detection after navigation.
- Handles retries and state transitions robustly.
*/
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "group_6_assignment_1/srv/detect_tables.hpp"
#include <future>
using namespace std::chrono_literals;
using DetectTables = group_6_assignment_1::srv::DetectTables;

#define SUBSCRIBER_QUEUE_SIZE 10
#define FEEDBACK_THROTTLE_TIMER_SEC 0.5
#define SERVICE_WAIT_TIMER_SEC 2
#define CHECK_READY_TIMER_SEC 1

enum class OrchState {PRE_NAVIGATING, NAVIGATING, IDLE};

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
        // Creates table detector client
        table_detector_client_ = this->create_client<DetectTables>("detect_tables");
      
        // Subscribing to GoalDetector publisher to get the goal position
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose",   
            SUBSCRIBER_QUEUE_SIZE,
            std::bind(&NavigationOrchestrator::goalCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "NavigationOrchestrator is ready and listening for goal_pose.");

        // state logic operates in timer thread.
        state_timer_ = this->create_wall_timer(1s, std::bind(&NavigationOrchestrator::stateTick, this));
        // enter initial state
        setState(OrchState::PRE_NAVIGATING);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Client<DetectTables>::SharedPtr table_detector_client_;
 
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
    
    rclcpp::TimerBase::SharedPtr state_timer_;
    OrchState current_state_;
    //goal_pose
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_received_;

    void setState(OrchState new_state) {current_state_ = new_state;}

    void stateTick() {
        switch (current_state_) { 
            case OrchState::PRE_NAVIGATING:
                //
                if (!navigate_to_pose_client_->wait_for_action_server(1s)) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Goal cannot be sent because NavigateToPose action server not available.");
                        return;
                } else if (goal_pose_received_) {
                    // we can send and we have a goal.
                    startGoalNavigation();
                }
                //
            break;
            case OrchState::NAVIGATING:
                //callbacks handle everything...

            break;
            case OrchState::IDLE:
                // do nothing..
            break;
        }
    }

    /*
        send action Navigation to Goal to nav2, setup action callbacks.
    */
    void startGoalNavigation()
    {

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *goal_pose_received_;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        //bind member function as feedback and result callback
        send_goal_options.feedback_callback =
            std::bind(&NavigationOrchestrator::feedbackCallback, this,
                    std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&NavigationOrchestrator::resultCallback, this,
                    std::placeholders::_1);

        send_goal_options.goal_response_callback = 
            std::bind(&NavigationOrchestrator::goalResponseCallback, this, 
                std::placeholders::_1);

        // Sends the goal to Nav2
        RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2");
        navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
        
       
    }

    void goalResponseCallback(GoalHandleNav::SharedPtr handle)
    {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2 action server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2 action server.");
            setState(OrchState::NAVIGATING);
        }
    }
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        
        // Collects the goal position if not already received
        
        if (current_state_ != OrchState::PRE_NAVIGATING) { return; }

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: frame_id='%s' pos(%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        // set received goal
        goal_pose_received_ = msg;

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
                //go backt to pre-navigating to try again
                setState(OrchState::PRE_NAVIGATING);
                break;
            case rclcpp_action::ResultCode::CANCELED:c
                RCLCPP_WARN(this->get_logger(), "Navigation anceled");
                // go back to pre-navigating to try again
                setState(OrchState::PRE_NAVIGATING);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Navigation error");
                // go back to pre-navigating to try again
                setState(OrchState::PRE_NAVIGATING);
                break;
        }
    }

    void callTableDetectionServer()
    {
        if (!table_detector_client_->wait_for_service(std::chrono::seconds(SERVICE_WAIT_TIMER_SEC))) {
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
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "TableDetectionServer succeeded: %s", response->message.c_str());
                        setState(OrchState::IDLE);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "TableDetectionServer reported failure: %s", response->message.c_str());
                        setState(OrchState::IDLE);
                    }
                }
                catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    setState(OrchState::IDLE);
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