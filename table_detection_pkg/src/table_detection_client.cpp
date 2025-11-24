#include <rclcpp/rclcpp.hpp>
#include "table_detection_pkg/srv/detect_tables.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TableDetectionClient : public rclcpp::Node {
public:
    TableDetectionClient() : Node("table_detection_client") {
        client_ = this->create_client<table_detection_pkg::srv::DetectTables>("detect_tables");
        
        RCLCPP_INFO(this->get_logger(), "Table Detection Client started");
    }
    
    void sendRequest() {
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
        
        // Create request
        auto request = std::make_shared<table_detection_pkg::srv::DetectTables::Request>();
        
        RCLCPP_INFO(this->get_logger(), "Sending table detection request...");
        
        // Send request asynchronously
        auto result_future = client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            
            auto result = result_future.get();
            
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "\n=== TABLE DETECTION RESULTS ===");
                RCLCPP_INFO(this->get_logger(), "Status: %s", result->message.c_str());
                RCLCPP_INFO(this->get_logger(), "Number of tables detected: %zu", 
                           result->table_positions.size());
                
                if (result->table_positions.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No tables detected!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "\nTable positions (in odom frame):");
                    for (size_t i = 0; i < result->table_positions.size(); i++) {
                        const auto& pos = result->table_positions[i];
                        RCLCPP_INFO(this->get_logger(), 
                                   "  Table %zu: x=%.3f, y=%.3f, z=%.3f",
                                   i + 1, pos.x, pos.y, pos.z);
                    }
                }
                RCLCPP_INFO(this->get_logger(), "===============================\n");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Detection failed: %s", 
                            result->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<table_detection_pkg::srv::DetectTables>::SharedPtr client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto client_node = std::make_shared<TableDetectionClient>();
    client_node->sendRequest();
    
    rclcpp::shutdown();
    return 0;
}