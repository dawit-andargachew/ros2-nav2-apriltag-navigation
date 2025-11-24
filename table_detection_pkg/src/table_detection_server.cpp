#include "rclcpp/rclcpp.hpp"
#include "table_detection_pkg/srv/detect_tables.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <vector>
#include <mutex>
#include <cmath>
#include <memory>

class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer() : Node("table_detection_server") {
        // Create service
        service_ = this->create_service<table_detection_pkg::srv::DetectTables>(
            "detect_tables",
            std::bind(&TableDetectionServer::detectTablesCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Create a subscriber for the /scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TableDetectionServer::scan_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Cylinder Detection Server is ready.");
    }

private:
    rclcpp::Service<table_detection_pkg::srv::DetectTables>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::mutex scan_mutex_;

    // Callback for the service
    void detectTablesCallback(
        const std::shared_ptr<table_detection_pkg::srv::DetectTables::Request> request,
        std::shared_ptr<table_detection_pkg::srv::DetectTables::Response> response) {

        (void)request;  // Mark the unused 'request' parameter as unused to avoid the warning

        int cylinder_count = scan_for_cylinders();
        RCLCPP_INFO(this->get_logger(), "Detected %d cylinders", cylinder_count);

        // Send the cylinder positions and success status
        response->success = (cylinder_count > 0);
        response->message = "Cylinder detection completed";
    }

    // Scan for cylinders
    int scan_for_cylinders() {
        if (!latest_scan_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available.");
            return 0;
        }

        std::lock_guard<std::mutex> lock(scan_mutex_);
        const std::vector<float>& ranges = latest_scan_->ranges;
        std::vector<std::vector<size_t>> clusters;
        std::vector<size_t> current_cluster;
        const float cluster_threshold = 0.3; // 30cm cluster threshold
        const float width_min = 0.05; // Minimum cylinder diameter (5 cm)
        const float width_max = 0.30; // Maximum cylinder diameter (30 cm)

        // Cluster the points based on range proximity
        for (size_t i = 0; i < ranges.size(); i++) {
            float range = ranges[i];
            if (range < latest_scan_->range_min || range > latest_scan_->range_max) {
                continue;
            }
            if (current_cluster.empty()) {
                current_cluster.push_back(i);
                continue;
            }

            float last_range = ranges[current_cluster.back()];
            if (std::abs(range - last_range) < cluster_threshold) {
                current_cluster.push_back(i);
            } else {
                clusters.push_back(current_cluster);
                current_cluster.clear();
                current_cluster.push_back(i);
            }
        }

        // Process each cluster
        int cylinder_count = 0;
        geometry_msgs::msg::PoseArray cylinder_poses;
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", clusters.size());  // Use %zu for size_t

        for (const auto& cluster : clusters) {
            if (cluster.size() < 2) {
                continue; // Too few points to form a cylinder
            }

            float range_first = ranges[cluster.front()];
            float range_last = ranges[cluster.back()];
            float angle_span = (cluster.size() - 1) * latest_scan_->angle_increment;
            float width = std::sqrt(range_first * range_first + range_last * range_last -
                                    2 * range_first * range_last * std::cos(angle_span));

            // Check if the width is within expected cylinder dimensions
            if (width >= width_min && width <= width_max) {
                cylinder_count++;
                float angle = latest_scan_->angle_min + (cluster.front() * latest_scan_->angle_increment);
                float range_median = ranges[cluster[cluster.size() / 2]];
                float x = range_median * std::cos(angle);
                float y = range_median * std::sin(angle);

                RCLCPP_INFO(this->get_logger(), "Cylinder detected at range: %.2f, angle: %.2f", range_median, angle);

                // Create pose for visualization
                geometry_msgs::msg::Pose cylinder_pose;
                cylinder_pose.position.x = x;
                cylinder_pose.position.y = y;
                cylinder_pose.position.z = 0.0;
                cylinder_pose.orientation.x = 0.0;
                cylinder_pose.orientation.y = 0.0;
                cylinder_pose.orientation.z = std::sin((angle + M_PI) / 2.0);
                cylinder_pose.orientation.w = std::cos((angle + M_PI) / 2.0);
                cylinder_poses.poses.push_back(cylinder_pose);
            }
        }

        // Publish the cylinder positions
        cylinder_poses.header.frame_id = latest_scan_->header.frame_id;
        cylinder_poses.header.stamp = this->now();
        RCLCPP_INFO(this->get_logger(), "Published %zu cylinder poses.", cylinder_poses.poses.size());  // Use %zu for size_t
        return cylinder_count;
    }

    // Callback for laser scan data
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = msg;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto server_node = std::make_shared<TableDetectionServer>();
    rclcpp::spin(server_node);
    rclcpp::shutdown();
    return 0;
}
