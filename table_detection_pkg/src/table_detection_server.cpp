#include "rclcpp/rclcpp.hpp"
#include "table_detection_pkg/srv/detect_tables.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <vector>
#include <mutex>
#include <memory>

class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer() : Node("table_detection_server") {
        // Create service to detect tables, which will trigger the detectTablesCallback
        service_ = this->create_service<table_detection_pkg::srv::DetectTables>(
            "detect_tables",
            std::bind(&TableDetectionServer::detectTablesCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Create a subscription to the /scan topic to receive LaserScan messages
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

    // Callback for the detect_tables service
    void detectTablesCallback(
        const std::shared_ptr<table_detection_pkg::srv::DetectTables::Request> request,
        std::shared_ptr<table_detection_pkg::srv::DetectTables::Response> response) {

        (void)request;  // Mark the unused 'request' parameter as unused to avoid the warning

        int cylinder_count = scan_for_cylinders();  // Call the scan_for_cylinders function
        RCLCPP_INFO(this->get_logger(), "Detected %d cylinders", cylinder_count);

        // Send the cylinder positions and success status as response
        response->success = (cylinder_count > 0);
        response->message = "Cylinder detection completed";
    }

    // Scan for cylinders based on the laser scan data
    int scan_for_cylinders() {
        if (!latest_scan_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available.");
            return 0;
        }

        std::lock_guard<std::mutex> lock(scan_mutex_);
        const std::vector<float>& ranges = latest_scan_->ranges;
        std::vector<std::vector<size_t>> clusters;  // To hold clusters of indices
        std::vector<size_t> current_cluster;        // A temporary cluster
        const float cluster_threshold = 0.3;       // 60cm cluster proximity threshold
        const float width_min = 0.20;              // Minimum cylinder diameter (20 cm)
        const float width_max = 0.45;              // Maximum cylinder diameter (45 cm)

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

        // 2. Filter clusters based on size and width, and calculate cylinder poses
        int cylinder_count = 0;
        geometry_msgs::msg::PoseArray cylinder_poses;
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", clusters.size());  // Use %zu for size_t

        for (const auto& cluster : clusters) {
            if (cluster.size() < 2) {
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
                cylinder_count++;  // Increment the count of detected cylinders

                // Calculate the position of the cylinder using the median point of the cluster
                float angle = latest_scan_->angle_min + (cluster.front() * latest_scan_->angle_increment);
                float range_median = ranges[cluster[cluster.size() / 2]];
                float x = range_median * std::cos(angle);
                float y = range_median * std::sin(angle);

                
                // Log the (x, y, z) coordinates
                RCLCPP_INFO(this->get_logger(), "Cylinder detected at range: %.2f, angle: %.2f, x: %.2f, y: %.2f", range_median, angle, x, y);

                // Create a Pose message for visualization in RViz
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

        // 4. Publish the cylinder positions for visualization in RViz
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
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto server_node = std::make_shared<TableDetectionServer>();  // Create and spin the TableDetectionServer node
    rclcpp::spin(server_node);
    rclcpp::shutdown();  // Shut down ROS2 when the node is finished
    return 0;
}
