#include <rclcpp/rclcpp.hpp>
#include "table_detection_pkg/srv/detect_tables.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CylinderDetectionClient : public rclcpp::Node {
public:
    CylinderDetectionClient() : Node("cylinder_detection_client") {
        client_ = this->create_client<table_detection_pkg::srv::DetectTables>("detect_tables");
        
        RCLCPP_INFO(this->get_logger(), "Cylinder Detection Client started");
    }
    
    bool sendRequest() {
        // Wait for service to be available
        RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        if (!client_->wait_for_service(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after 10 seconds");
            return false;
        }
        
        // Create request (empty for this service)
        auto request = std::make_shared<table_detection_pkg::srv::DetectTables::Request>();
        
        RCLCPP_INFO(this->get_logger(), "Sending cylinder detection request...");
        
        // Send request asynchronously
        auto result_future = client_->async_send_request(request);
        
        // Wait for result with timeout
        auto status = result_future.wait_for(10s);
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out after 10 seconds");
            return false;
        }
        
        try {
            auto result = result_future.get();
            displayResults(result);
            return result->success;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            return false;
        }
    }
    
private:
    rclcpp::Client<table_detection_pkg::srv::DetectTables>::SharedPtr client_;
    
    void displayResults(const std::shared_ptr<table_detection_pkg::srv::DetectTables::Response>& result) {
        RCLCPP_INFO(this->get_logger(), "\n=== CYLINDER DETECTION RESULTS ===");
        RCLCPP_INFO(this->get_logger(), "Status: %s", result->success ? "SUCCESS" : "FAILED");
        RCLCPP_INFO(this->get_logger(), "Message: %s", result->message.c_str());

        // Print the number of cylinders detected
        size_t num_cylinders = result->table_positions.size();
        RCLCPP_INFO(this->get_logger(), "Number of cylinders detected: %zu", num_cylinders);
        
        if (num_cylinders == 0) {
            RCLCPP_WARN(this->get_logger(), "No cylinders detected!");
        } else {
            RCLCPP_INFO(this->get_logger(), "\nCylinder positions (in odom frame):");
            for (size_t i = 0; i < num_cylinders; i++) {
                const auto& pos = result->table_positions[i];
                RCLCPP_INFO(this->get_logger(), 
                           "  Cylinder %zu: x=%.3f, y=%.3f, z=%.3f",
                           i + 1, pos.x, pos.y, pos.z);
            }
        }
        RCLCPP_INFO(this->get_logger(), "=================================\n");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto client_node = std::make_shared<CylinderDetectionClient>();
    
    // Send request and check result
    bool success = client_node->sendRequest();
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}
