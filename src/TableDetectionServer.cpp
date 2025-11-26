/* src/TableDetectionServer.cpp

It's a service for detecting the three cylinderical tables

Input:
    - It uses the robots lidar to scan its sourronding and it look for circuar edges that resemble a cylinder
    - then cluter the points from the lidar scan and search for circular edges
    - after filtering out circular edges, these coordinates chagned to odom refernce frame 

Output
    - it gives the respective coordinates of each cylinderical table relative to odom reference fram

*/


#include "rclcpp/rclcpp.hpp"
#include "group_6_assignment_1/srv/detect_tables.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



// Constants need to detect cylinder around the robot

#define SCAN_SUBSCRIPTION_QUEUE_SIZE 10 // Queue size for the subscription
#define MIN_CLUSTER_SIZE 2
#define MIN_CYLINDER_WIDTH 0.05  // HINT: must be > april_tag dimension (0.05 X 0.05 (M) -> 5cm) (The cylinders are bigger), 
#define MAX_CYLINDER_WIDTH 0.5  // Maximum cylinder diameter ( half a meter )
#define CLUSTER_PROXIMITY_THRESHOLD 0.25  // cluster proximity  25cm
#define MAX_CYLINDER_DETECTION_RANGE 3 // 3 meters

class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer() : Node("table_detection_server") {
        // Create service to detect tables, which will trigger the detectTablesCallback
        service_ = this->create_service<group_6_assignment_1::srv::DetectTables>(
            "detect_tables",
            std::bind(&TableDetectionServer::detectTablesCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Create a subscription to the /scan topic to receive LaserScan messages
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", SCAN_SUBSCRIPTION_QUEUE_SIZE,
            std::bind(&TableDetectionServer::scan_callback, this, std::placeholders::_1));

        //create publisher to publish cylinder poses
        cylinder_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "cylinder_poses", SCAN_SUBSCRIPTION_QUEUE_SIZE);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "Cylinder Detection Server is ready.");
    }

private:
    rclcpp::Service<group_6_assignment_1::srv::DetectTables>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cylinder_pose_publisher_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::mutex scan_mutex_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Callback for the detect_tables service
    void detectTablesCallback(
        const std::shared_ptr<group_6_assignment_1::srv::DetectTables::Request> request,
        std::shared_ptr<group_6_assignment_1::srv::DetectTables::Response> response) {

        (void)request;  // Mark the unused 'request' parameter as unused to avoid the warning

        int cylinder_count = scan_for_cylinders();  // Call the scan_for_cylinders function
        RCLCPP_INFO(this->get_logger(), "Detected %d cylinders", cylinder_count);

        // Send the cylinder positions and success status as response
        response->success = (cylinder_count > 0);
        response->message = "Cylinder detection completed";
    }

    // Scan for cylinders based on the laser scan data
    int scan_for_cylinders() {
        // lock while we access latest_scan_
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!latest_scan_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available.");
            return 0;
        }
        const std::vector<float>& ranges = latest_scan_->ranges;
        std::vector<std::vector<size_t>> clusters;  // To hold clusters of indices
        std::vector<size_t> current_cluster;        // A temporary cluster
        const float cluster_threshold = CLUSTER_PROXIMITY_THRESHOLD;       
        const float width_min = MIN_CYLINDER_WIDTH;              
        const float width_max = MAX_CYLINDER_WIDTH;             

    
        // 1. Get all clusters: Group laser scan points that are close to each other into clusters
        for (size_t i = 0; i < ranges.size(); i++) {
            float range = ranges[i];
            if (range < latest_scan_->range_min || range > latest_scan_->range_max) {
                continue;  // Ignore out-of-range points
            }
            if (current_cluster.empty()) {
                current_cluster.push_back(i);
                continue;
            }

            float last_range = ranges[current_cluster.back()];
            // Check if the current point is close enough to the previous one to be part of the same cluster
            if (std::abs(range - last_range) < cluster_threshold) {
                current_cluster.push_back(i);
            } else {
                clusters.push_back(current_cluster);  // Save the current cluster
                current_cluster.clear();  // Start a new cluster
                current_cluster.push_back(i);  // Add the current point to the new cluster
            }
        }
        // push the last cluster if non-empty
        if (!current_cluster.empty()) {
            clusters.push_back(current_cluster);
        }

        // 2. Filter clusters based on size and width, and calculate cylinder poses
        int cylinder_count = 0;
        geometry_msgs::msg::PoseArray cylinder_poses;
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", clusters.size());  // Use %zu for size_t

        for (const auto& cluster : clusters) {
            if (cluster.size() < MIN_CLUSTER_SIZE) {
                continue; // Too few points to form a cylinder (cluster size should be >= 2)
            }

            // Calculate the width of the cluster by measuring the distance between the first and last point
            float range_first = ranges[cluster.front()];
            float range_last = ranges[cluster.back()];
            float angle_span = (cluster.size() - 1) * latest_scan_->angle_increment;
            float width = std::sqrt(range_first * range_first + range_last * range_last -
                                    2 * range_first * range_last * std::cos(angle_span));

            // 3. Filter clusters based on expected cylinder dimensions (width)
            if (width >= width_min && width <= width_max) {
                // Calculate the position of the cylinder using the median point of the cluster
                size_t median_idx = cluster[cluster.size() / 2];
                float angle = latest_scan_->angle_min + (median_idx * latest_scan_->angle_increment);
                float range_median = ranges[median_idx];
                float x = range_median * std::cos(angle);
                float y = range_median * std::sin(angle);

                //filter range that are outside the range
                if (range_median > MAX_CYLINDER_DETECTION_RANGE) {
                    continue;
                }
                
                // Create a Pose message for visualization in RViz
                //this reference frame is "base_link"
                geometry_msgs::msg::Pose cylinder_pose;
                cylinder_pose.position.x = x;
                cylinder_pose.position.y = y;
                cylinder_pose.position.z = 0.0;
                cylinder_pose.orientation.x = 0.0;
                cylinder_pose.orientation.y = 0.0;
                cylinder_pose.orientation.z = 0.0;
                cylinder_pose.orientation.w = 1.0;


                // transform to "odom" using tf2; only add pose on success
                bool transformed = false;
                try {
                    if (tf_buffer_->canTransform("odom", latest_scan_->header.frame_id, tf2::TimePointZero)) {
                        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                            "odom",  // target frame
                            latest_scan_->header.frame_id,  // source frame
                            tf2::TimePointZero);  // get the latest available transform

                        geometry_msgs::msg::PoseStamped pose_in, pose_out;
                        pose_in.header.frame_id = latest_scan_->header.frame_id;
                        pose_in.header.stamp = latest_scan_->header.stamp;
                        pose_in.pose = cylinder_pose;

                        tf2::doTransform(pose_in, pose_out, transform_stamped);
                        cylinder_pose = pose_out.pose;  // Update to transformed pose
                        transformed = true;
                    } else {
                        RCLCPP_DEBUG(this->get_logger(), "Transform not available yet: %s -> odom", latest_scan_->header.frame_id.c_str());
                    }
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Could not transform cylinder pose: %s", ex.what());
                }

                if (transformed) {
                    cylinder_poses.poses.push_back(cylinder_pose);
                    cylinder_count++;  // Increment only when pose successfully transformed and queued
                }
            }
        }
        // 4. Publish the cylinder positions for visualization in RViz
        // Poses were transformed to 'odom', so set header accordingly and use scan timestamp
        cylinder_poses.header.frame_id = "odom";
        cylinder_poses.header.stamp = latest_scan_->header.stamp;
        
        cylinder_pose_publisher_->publish(cylinder_poses);
        return cylinder_count;
    }

    // Callback for laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = msg;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto server_node = std::make_shared<TableDetectionServer>();  // Create and spin the TableDetectionServer node
    rclcpp::spin(server_node);
    rclcpp::shutdown();  // Shut down ROS2 when the node is finished
    return 0;
}
