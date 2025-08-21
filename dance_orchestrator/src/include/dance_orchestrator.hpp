#pragma once

#include <memory>
#include <atomic>
#include <future>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "dance_sequence.hpp"
#include "dance_move.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "common/ros2_robot_state_client.h"
#include "common/ros2_sport_client.h"

/**
 * @brief Main orchestrator class for executing dance sequences
 */
class DanceOrchestrator : public rclcpp::Node {
private:
    // Client instances
    std::unique_ptr<SportClient> sport_client_;
    std::unique_ptr<RobotStateClient> state_client_;
    
    // Current sequence management
    std::unique_ptr<DanceSequence> current_sequence_;
    
    // Response monitoring
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
    std::atomic<bool> execution_active_{false};
    std::atomic<int32_t> current_api_id_{0};
    
    // Sequence execution state
    size_t current_move_index_{0};
    
    // Timing and control
    rclcpp::TimerBase::SharedPtr execution_timer_;
    rclcpp::TimerBase::SharedPtr delay_timer_;
    std::atomic<bool> emergency_stop_requested_{false};

public:
    explicit DanceOrchestrator(const std::string& node_name = "dance_orchestrator");
    ~DanceOrchestrator() = default;

    // Sequence management
    void setCurrentSequence(std::unique_ptr<DanceSequence> sequence);
    DanceSequence* getCurrentSequence() const { return current_sequence_.get(); }
    void clearCurrentSequence() { current_sequence_.reset(); }

    // Execution methods
    bool executeSequence(const DanceSequence& sequence);
    bool executeMove(const DanceMove& move);
    void emergencyStop();
    
    // Status methods
    bool isExecutionActive() const { return execution_active_.load(); }
    bool isEmergencyStopRequested() const { return emergency_stop_requested_.load(); }
    
    // Safety and monitoring
    bool performSafetyCheck();

private:
    // Internal methods
    void setupResponseMonitoring();
    void handleSportResponse(const unitree_api::msg::Response::SharedPtr msg);
    void resetExecutionState();
    bool sendMoveRequest(const DanceMove& move);
    void executeNextMove();
    void scheduleDelayedExecution(float delay_seconds);
    
    // Utility methods
    void logInfo(const std::string& message);
    void logError(const std::string& message);
    void logWarn(const std::string& message);
    
    // SportClient integration
    bool callSportClientMethod(const DanceMove& move, unitree_api::msg::Request& req);
};
