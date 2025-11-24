#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "table_detection_pkg/srv/detect_tables.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

struct Point2D {
    double x, y;
};

struct Cluster {
    std::vector<Point2D> points;
    Point2D centroid;
    double radius;
    
    void calculateCentroid() {
        if (points.empty()) return;
        centroid.x = 0;
        centroid.y = 0;
        for (const auto& p : points) {
            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= points.size();
        centroid.y /= points.size();
    }
    
    void calculateRadius() {
        if (points.empty()) return;
        radius = 0;
        for (const auto& p : points) {
            double dist = std::sqrt(std::pow(p.x - centroid.x, 2) + 
                                   std::pow(p.y - centroid.y, 2));
            radius = std::max(radius, dist);
        }
    }
};

class TableDetectionServer : public rclcpp::Node {
public:
    TableDetectionServer() : Node("table_detection_server") {
        // Create service
        service_ = this->create_service<table_detection_pkg::srv::DetectTables>(
            "detect_tables",
            std::bind(&TableDetectionServer::detectTablesCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Subscribe to LIDAR
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&TableDetectionServer::scanCallback, this, std::placeholders::_1));
        
        // TF2 setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Parameters
        this->declare_parameter("cluster_threshold", 0.15);
        this->declare_parameter("min_table_radius", 0.25);
        this->declare_parameter("min_table_points", 20);
        this->declare_parameter("max_table_radius", 0.75);
        
        cluster_threshold_ = this->get_parameter("cluster_threshold").as_double();
        min_table_radius_ = this->get_parameter("min_table_radius").as_double();
        min_table_points_ = this->get_parameter("min_table_points").as_int();
        max_table_radius_ = this->get_parameter("max_table_radius").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Table Detection Server started");
        RCLCPP_INFO(this->get_logger(), "Parameters - Cluster: %.2f, MinRadius: %.2f, MinPoints: %d",
                    cluster_threshold_, min_table_radius_, min_table_points_);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }
    

    // here we're gonn
    void detectTablesCallback(
        const std::shared_ptr<table_detection_pkg::srv::DetectTables::Request> request,
        std::shared_ptr<table_detection_pkg::srv::DetectTables::Response> response) {
        
        (void)request; // Unused
        
        RCLCPP_INFO(this->get_logger(), "Table detection service called");
        
        if (!latest_scan_) {
            response->success = false;
            response->message = "No LIDAR data available";
            RCLCPP_WARN(this->get_logger(), "No LIDAR data available");
            return;
        }
        
        // Convert scan to cartesian points
        std::vector<Point2D> scan_points;
        convertScanToPoints(latest_scan_, scan_points);
        
        RCLCPP_INFO(this->get_logger(), "Converted %zu scan points", scan_points.size());
        
        // Cluster points
        std::vector<Cluster> clusters = clusterPoints(scan_points);
        RCLCPP_INFO(this->get_logger(), "Found %zu clusters", clusters.size());
        
        // Filter for tables
        std::vector<Cluster> tables;
        for (auto& cluster : clusters) {
            cluster.calculateCentroid();
            cluster.calculateRadius();
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Cluster: points=%zu, radius=%.3f", 
                        cluster.points.size(), cluster.radius);
            
            if (isTable(cluster)) {
                tables.push_back(cluster);
                RCLCPP_INFO(this->get_logger(), 
                           "Table detected: center=(%.2f, %.2f), radius=%.2f, points=%zu",
                           cluster.centroid.x, cluster.centroid.y, 
                           cluster.radius, cluster.points.size());
            }
        }
        
        // Transform to odom frame and populate response
        for (const auto& table : tables) {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = latest_scan_->header.frame_id;
            point_in.header.stamp = this->now();
            point_in.point.x = table.centroid.x;
            point_in.point.y = table.centroid.y;
            point_in.point.z = 0.0;
            
            try {
                tf_buffer_->transform(point_in, point_out, "odom", 
                                     tf2::durationFromSec(1.0));
                
                geometry_msgs::msg::Point table_pos;
                table_pos.x = point_out.point.x;
                table_pos.y = point_out.point.y;
                table_pos.z = point_out.point.z;
                response->table_positions.push_back(table_pos);
                
                RCLCPP_INFO(this->get_logger(), 
                           "Table in odom frame: (%.2f, %.2f, %.2f)",
                           table_pos.x, table_pos.y, table_pos.z);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 transform failed: %s", ex.what());
            }
        }
        
        response->success = true;
        response->message = "Detected " + std::to_string(tables.size()) + " tables";
        
        RCLCPP_INFO(this->get_logger(), "Detection complete: %zu tables found", tables.size());
    }
    
    void convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                            std::vector<Point2D>& points) {
        points.clear();
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            
            // Filter invalid readings
            if (range >= scan->range_min && range <= scan->range_max) {
                Point2D p;
                p.x = range * std::cos(angle);
                p.y = range * std::sin(angle);
                points.push_back(p);
            }
            
            angle += scan->angle_increment;
        }
    }
    
    std::vector<Cluster> clusterPoints(const std::vector<Point2D>& points) {
        std::vector<Cluster> clusters;
        std::vector<bool> assigned(points.size(), false);
        
        for (size_t i = 0; i < points.size(); i++) {
            if (assigned[i]) continue;
            
            Cluster cluster;
            cluster.points.push_back(points[i]);
            assigned[i] = true;
            
            // Grow cluster
            for (size_t j = 0; j < cluster.points.size(); j++) {
                for (size_t k = 0; k < points.size(); k++) {
                    if (assigned[k]) continue;
                    
                    double dist = distance(cluster.points[j], points[k]);
                    if (dist < cluster_threshold_) {
                        cluster.points.push_back(points[k]);
                        assigned[k] = true;
                    }
                }
            }
            
            if (cluster.points.size() >= 5) { // Minimum points per cluster
                clusters.push_back(cluster);
            }
        }
        
        return clusters;
    }
    
    bool isTable(const Cluster& cluster) {
        // Tables should have:
        // 1. Sufficient radius (larger than cubes)
        // 2. Sufficient number of points
        // 3. Not too large (filter out walls)
        
        bool has_min_radius = cluster.radius >= min_table_radius_;
        bool has_max_radius = cluster.radius <= max_table_radius_;
        bool has_min_points = cluster.points.size() >= static_cast<size_t>(min_table_points_);
        
        return has_min_radius && has_max_radius && has_min_points;
    }
    
    double distance(const Point2D& p1, const Point2D& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }
    
    rclcpp::Service<table_detection_pkg::srv::DetectTables>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    double cluster_threshold_;
    double min_table_radius_;
    int min_table_points_;
    double max_table_radius_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableDetectionServer>());
    rclcpp::shutdown();
    return 0;
}