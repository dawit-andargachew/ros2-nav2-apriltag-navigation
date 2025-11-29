/**
 * Node: table_detection_server
 *
 * Purpose:
 *   - Detects cylindrical tables in the robot's environment using lidar scan data.
 *   - Clusters laser scan points, filters clusters by geometric constraints, and identifies cylinders.
 *   - Publishes the poses of detected cylinders in the "odom" reference frame.
 *
 * Inputs:
 *   - /scan (sensor_msgs::msg::LaserScan): continuous laser scan data from the robot's lidar.
 *   - Service request: triggers detection of tables from the latest scan.
 *
 * Parameters:
 *   - min_cluster_size: minimum number of points for a valid cluster
 *   - min_cylinder_width / max_cylinder_width: constraints on cylinder size
 *   - cluster_proximity_threshold: max distance between points in a cluster
 *   - max_cylinder_detection_range: ignore objects beyond this range
 *
 * Outputs:
 *   - Service response: success flag and message
 *   - /cylinder_poses (geometry_msgs::msg::PoseArray): poses of detected cylindrical tables
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

#define SCAN_SUBSCRIPTION_QUEUE_SIZE 10     // Queue size for laser scan subscription

class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer()
        : Node("table_detection_server") {

        // Load detection parameters from YAML file
        MIN_CLUSTER_SIZE = this->declare_parameter< int>("min_cluster_size", 2);
        MIN_CYLINDER_WIDTH = this->declare_parameter<double>("min_cylinder_width", 0.05);
        MAX_CYLINDER_WIDTH = this->declare_parameter<double>("max_cylinder_width", 0.5);
        CLUSTER_PROXIMITY_THRESHOLD = this->declare_parameter<double>("cluster_proximity_threshold", 0.25);
        MAX_CYLINDER_DETECTION_RANGE = this->declare_parameter<double>("max_cylinder_detection_range", 3.0);

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
        RCLCPP_INFO(this->get_logger(), "Table Detection Server is ready.");
    }

private:

    // Detection parameters (loaded from YAML configuration)
    long unsigned int MIN_CLUSTER_SIZE;      // Minimum number of points for a valid cluster
    double MIN_CYLINDER_WIDTH;               // Minimum cylinder width (in meters)
    double MAX_CYLINDER_WIDTH;               // Maximum cylinder width (in meters)
    double CLUSTER_PROXIMITY_THRESHOLD;      // Threshold for proximity to consider points in the same cluster (in meters)
    double MAX_CYLINDER_DETECTION_RANGE;     // Maximum range for cylinder detection (in meters)

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
        response->message = "Table detection completed! \n\n";
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
        RCLCPP_INFO(this->get_logger(), "\n Detecting %zu cylinder table poses in frame '%s'. \n", cylinder_poses.poses.size(), cylinder_poses.header.frame_id.c_str());

        // Print coordinates of each detected cylinder
        for (size_t i = 0; i < cylinder_poses.poses.size(); i++) {
            RCLCPP_INFO(this->get_logger(), 
                        "  Table %zu: x=%.3f, y=%.3f", 
                        i + 1, 
                        cylinder_poses.poses[i].position.x, 
                        cylinder_poses.poses[i].position.y);
        }
        
        RCLCPP_INFO(this->get_logger(), "\n\n");
        
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
