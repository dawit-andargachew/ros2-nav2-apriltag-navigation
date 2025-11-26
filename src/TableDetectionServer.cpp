/* src/TableDetectionServer.cpp

This service detects cylindrical tables in the robot's environment by processing
laser scan data from the robot's lidar. It clusters points, evaluates cylindrical
shape characteristics, filters based on geometric constraints, and outputs the
PoseArray of detected cylinders in the odom reference frame.
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

// Constants for detecting cylinders
#define SCAN_SUBSCRIPTION_QUEUE_SIZE 10     // Queue size for laser scan subscription
#define MIN_CLUSTER_SIZE 2                  // Minimum number of points for a valid cluster
#define MIN_CYLINDER_WIDTH 0.05             // Minimum cylinder width (in meters), must be > april_tag dimension (0.05 X 0.05 -> 5cm), cylinders are bigger
#define MAX_CYLINDER_WIDTH 0.5              // Maximum cylinder width (in meters)
#define CLUSTER_PROXIMITY_THRESHOLD 0.25    // Threshold for proximity to consider points in the same cluster (in meters)
#define MAX_CYLINDER_DETECTION_RANGE 3      // Maximum range for cylinder detection (in meters)


class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer()
        : Node("table_detection_server") {

        // Create detection service
        service_ = this->create_service<group_6_assignment_1::srv::DetectTables>(
            "detect_tables",
            std::bind(&TableDetectionServer::detectTablesCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Subscribe to /scan for continuous laser data
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", SCAN_SUBSCRIPTION_QUEUE_SIZE,
            std::bind(&TableDetectionServer::scan_callback, this, std::placeholders::_1));

        // Publisher for detected cylinder poses (odom frame)
        cylinder_pose_publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseArray>("cylinder_poses",
                                                                  SCAN_SUBSCRIPTION_QUEUE_SIZE);

        // TF2 listener for odom ← laser transforms
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

    // Service callback — triggers processing of the latest scan
    void detectTablesCallback(
        const std::shared_ptr<group_6_assignment_1::srv::DetectTables::Request> request,
        std::shared_ptr<group_6_assignment_1::srv::DetectTables::Response> response) {

        (void)request;

        int cylinder_count = scanForCylinders();
        response->success = (cylinder_count > 0);
        response->message = "Cylinder detection completed";
    }

    // Main scan processing routine
    int scanForCylinders() {
        std::lock_guard<std::mutex> lock(scan_mutex_);

        if (!latest_scan_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available.");
            return 0;
        }

        const std::vector<float>& ranges = latest_scan_->ranges;
        std::vector<std::vector<size_t>> clusters;

        // 1. Cluster scan points based on proximity
        groupPointsIntoClusters(ranges, clusters);

        // 2. Evaluate clusters and detect cylinders
        return processClusters(ranges, clusters);
    }

    // Groups scan points into clusters based on range proximity
    void groupPointsIntoClusters(const std::vector<float>& ranges,
                                 std::vector<std::vector<size_t>>& clusters) 
    {
        std::vector<size_t> current_cluster;

        for (size_t i = 0; i < ranges.size(); i++) {
            float range = ranges[i];

            // Skip invalid measurements
            if (range < latest_scan_->range_min || range > latest_scan_->range_max)
                continue;

            // Start new cluster when gap exceeds threshold
            if (current_cluster.empty() ||
                std::abs(range - ranges[current_cluster.back()]) < CLUSTER_PROXIMITY_THRESHOLD) {
                current_cluster.push_back(i);
            } else {
                clusters.push_back(current_cluster);
                current_cluster.clear();
                current_cluster.push_back(i);
            }
        }

        if (!current_cluster.empty())
            clusters.push_back(current_cluster);
    }

    // Evaluates each cluster and identifies cylinders
    int processClusters(const std::vector<float>& ranges,
                        const std::vector<std::vector<size_t>>& clusters) 
    {
        int cylinder_count = 0;
        geometry_msgs::msg::PoseArray cylinder_poses;

        for (const auto& cluster : clusters) {

            if (cluster.size() < MIN_CLUSTER_SIZE)
                continue;

            // Estimate cluster width (physical size)
            float width = calculateClusterWidth(ranges, cluster);

            // Filter out noise or oversized objects
            if (width < MIN_CYLINDER_WIDTH || width > MAX_CYLINDER_WIDTH)
                continue;

            // Approximate cylinder location using midpoint of cluster
            geometry_msgs::msg::Pose cylinder_pose = calculateCylinderPose(ranges, cluster);

            // Filter by max detection distance to reduce false positives
            size_t median_idx = cluster[cluster.size() / 2];
            float range_median = ranges[median_idx];
            if (range_median > MAX_CYLINDER_DETECTION_RANGE)
                continue;

            // Transform pose to odom frame
            if (transformPoseToOdom(cylinder_pose)) {
                cylinder_poses.poses.push_back(cylinder_pose);
                cylinder_count++;
            }
        }

        // Publish detected cylinder poses
        cylinder_poses.header.frame_id = "odom";
        cylinder_poses.header.stamp = latest_scan_->header.stamp;
        cylinder_pose_publisher_->publish(cylinder_poses);

        return cylinder_count;
    }

    // Calculates width of a cluster using law of cosines
    float calculateClusterWidth(const std::vector<float>& ranges,
                                const std::vector<size_t>& cluster) 
    {
        float r1 = ranges[cluster.front()];
        float r2 = ranges[cluster.back()];
        float angle_span = (cluster.size() - 1) * latest_scan_->angle_increment;

        return std::sqrt(r1*r1 + r2*r2 - 2*r1*r2*std::cos(angle_span));
    }

    // Computes cylinder pose in the laser frame
    geometry_msgs::msg::Pose calculateCylinderPose(const std::vector<float>& ranges,
                                                   const std::vector<size_t>& cluster) 
    {
        size_t mid = cluster[cluster.size() / 2];  // midpoint represents cylinder center
        float angle = latest_scan_->angle_min + mid * latest_scan_->angle_increment;
        float r = ranges[mid];

        geometry_msgs::msg::Pose pose;
        pose.position.x = r * std::cos(angle);
        pose.position.y = r * std::sin(angle);
        pose.position.z = 0.0;

        pose.orientation.w = 1.0;
        return pose;
    }

    // Transforms pose from laser → odom frame
    bool transformPoseToOdom(geometry_msgs::msg::Pose& pose) {
        try {
            if (tf_buffer_->canTransform("odom", latest_scan_->header.frame_id,
                                         tf2::TimePointZero)) {

                geometry_msgs::msg::TransformStamped tf =
                    tf_buffer_->lookupTransform("odom",
                                                latest_scan_->header.frame_id,
                                                tf2::TimePointZero);

                geometry_msgs::msg::PoseStamped p_in, p_out;
                p_in.header.frame_id = latest_scan_->header.frame_id;
                p_in.header.stamp = latest_scan_->header.stamp;
                p_in.pose = pose;

                tf2::doTransform(p_in, p_out, tf);
                pose = p_out.pose;
                return true;
            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not transform cylinder pose: %s", ex.what());
        }
        return false;
    }

    // Stores incoming laser scans
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        latest_scan_ = msg;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TableDetectionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
