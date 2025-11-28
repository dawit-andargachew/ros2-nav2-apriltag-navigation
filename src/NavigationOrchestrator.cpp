/* 
Node: navigation_orchestrator

Input: 
-
Output:
-

26th nov 2025
- adding lifecycle service client to the orchestrator
- to check the state of the nav2 stack
- so we can only send goals when nav2 is active


*/

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "group_6_assignment_1/srv/detect_tables.hpp"
#include <lifecycle_msgs/srv/get_state.hpp>
#include <atomic>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;
using DetectTables = group_6_assignment_1::srv::DetectTables;

#define SUBSCRIBER_QUEUE_SIZE 10
#define FEEDBACK_THROTTLE_TIMER_SEC 0.5
#define SERVICE_WAIT_TIMER_SEC 2
#define NAV_LIFECYCLE_NODE_NAME "lifecycle_manager_navigation"
#define CHECK_READY_TIMER_SEC 1
class NavigationOrchestrator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationOrchestrator() : Node("navigation_orchestrator")
    {   
        //CLIENTS 
        // Creates navigation action client
        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, 
            "navigate_to_pose"
        );
      
        // Creates table detector client
        table_detector_client_ = this->create_client<DetectTables>("detect_tables");
        
        // Creates lifecycle service client to monitor nav2 state
        nav_lifecycle_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
            "/" NAV_LIFECYCLE_NODE_NAME "/get_state"
        );

        //SUBSCRIBERS
        // Subscribing to GoalDetector publisher to get the goal position
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose",   
            SUBSCRIBER_QUEUE_SIZE,
            std::bind(&NavigationOrchestrator::goalCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "NavigationOrchestrator is ready and listening for goal_pose.");

        // start non-blocking readiness poller
        waitSystemReady();

    }

    void waitSystemReady() {
        RCLCPP_INFO(this->get_logger(), "Starting non-blocking Nav2 readiness poller...");

        ready_timer_ = this->create_wall_timer(
            std::chrono::seconds(CHECK_READY_TIMER_SEC),
            [this]() {
                // if already ready, stop timer and return
                if (system_ready_.load()) {
                    if (ready_timer_) {
                        ready_timer_->cancel();
                    }
                    return;
                }

                // non-blocking service availability check
                if (!nav_lifecycle_client_->service_is_ready()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Nav2 lifecycle manager service not available, waiting...");
                    return;
                }

                auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
                // async request with lambda callback; do not block inside timer callback
                nav_lifecycle_client_->async_send_request(
                    request,
                    [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                        try {
                            auto response = future.get();
                            if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                                RCLCPP_INFO(this->get_logger(), "Nav2 is ACTIVE.");
                                system_ready_.store(true);
                                if (ready_timer_) {
                                    ready_timer_->cancel();
                                }
                            } else {
                                // keep system_ready_ false
                                system_ready_.store(false);
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "Nav2 not ACTIVE yet (state id: %u), waiting...", response->current_state.id);
                            }
                        } catch (const std::exception & e) {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Failed to query Nav2 lifecycle state: %s", e.what());
                        }
                    }
                );
            }
        );
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Client<DetectTables>::SharedPtr table_detector_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr nav_lifecycle_client_;
    
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
    
    //atomic because used in timer callback
    std::atomic_bool system_ready_{false};
    
    rclcpp::TimerBase::SharedPtr ready_timer_;

    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        
        if (!system_ready_.load()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Goal ignored: system not ready yet"
            );
            return;
        }

        // Collects the goal position
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal: frame_id='%s' pos(%.2f, %.2f, %.2f)",
            msg->header.frame_id.c_str(),
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        if (!navigate_to_pose_client_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Goal cannot be sent because NavigateToPose action server not available.");
            return;
        }
    
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *msg;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // async function handle response callback
        // Note: rclcpp_action may expect either a direct GoalHandle shared_ptr or a future depending on API;
        // use the signature that accepts a GoalHandle shared_ptr for compatibility with this ROS 2 version.
        send_goal_options.goal_response_callback =
        [this](GoalHandleNav::SharedPtr handle) {
            if (!handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            }
        };
        //bind member function as feedback and result callback
        send_goal_options.feedback_callback =
            std::bind(&NavigationOrchestrator::feedbackCallback, this,
                    std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&NavigationOrchestrator::resultCallback, this,
                    std::placeholders::_1);

        // Sends the goal to Nav2
        RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2");
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
                    } else {
                        RCLCPP_WARN(this->get_logger(), "TableDetectionServer reported failure: %s", response->message.c_str());
                    }
                }
                catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            }
        );
    }


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);   
    auto node = std::make_shared<NavigationOrchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}