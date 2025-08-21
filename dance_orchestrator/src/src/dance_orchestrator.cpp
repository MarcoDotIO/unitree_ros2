#include "dance_orchestrator.hpp"
#include <chrono>
#include <thread>

DanceOrchestrator::DanceOrchestrator(const std::string& node_name) 
    : rclcpp::Node(node_name) {
    
    // Initialize clients
    sport_client_ = std::make_unique<SportClient>(this);
    state_client_ = std::make_unique<RobotStateClient>(this);
    
    // Setup response monitoring
    setupResponseMonitoring();
    
    logInfo("Dance Orchestrator initialized");
}

void DanceOrchestrator::setupResponseMonitoring() {
    response_sub_ = create_subscription<unitree_api::msg::Response>(
        "/api/sport/response", 10,
        [this](const unitree_api::msg::Response::SharedPtr msg) {
            handleSportResponse(msg);
        });
}

void DanceOrchestrator::handleSportResponse(const unitree_api::msg::Response::SharedPtr msg) {
    if (!execution_active_.load()) {
        return;
    }
    
    // Check if this response matches the current move we're waiting for
    if (msg->header.identity.api_id == current_api_id_.load()) {
        if (msg->header.status.code == 0) {
            logInfo("Received successful response for API ID: " + std::to_string(msg->header.identity.api_id));
            
            // Check if we need to apply delay before next move
            if (current_sequence_ && current_move_index_ > 0) {
                const auto& completed_move = current_sequence_->getMoves()[current_move_index_ - 1];
                if (completed_move.delay_after > 0.0f) {
                    logInfo("Applying delay of " + std::to_string(completed_move.delay_after) + " seconds");
                    scheduleDelayedExecution(completed_move.delay_after);
                    return;
                }
            }
            
            // No delay needed, execute next move immediately
            executeNextMove();
        } else {
            logError("Received error response for API ID: " + std::to_string(msg->header.identity.api_id) + 
                    ", error code: " + std::to_string(msg->header.status.code));
            // Move failed, stop sequence execution
            execution_active_.store(false);
            logError("Sequence execution failed due to move error");
        }
    }
}

void DanceOrchestrator::setCurrentSequence(std::unique_ptr<DanceSequence> sequence) {
    if (execution_active_.load()) {
        logWarn("Cannot change sequence while execution is active");
        return;
    }
    current_sequence_ = std::move(sequence);
    logInfo("Current sequence set: " + (current_sequence_ ? current_sequence_->getName() : "none"));
}

bool DanceOrchestrator::executeSequence(const DanceSequence& sequence) {
    if (execution_active_.load()) {
        logError("Execution already active");
        return false;
    }
    
    if (sequence.getMoves().empty()) {
        logError("Cannot execute empty sequence");
        return false;
    }
    
    // Perform safety check before execution
    if (!performSafetyCheck()) {
        logError("Safety check failed, aborting sequence");
        return false;
    }
    
    // Set up sequence execution state
    current_sequence_ = std::make_unique<DanceSequence>(sequence);
    current_move_index_ = 0;
    execution_active_.store(true);
    emergency_stop_requested_.store(false);
    
    logInfo("Starting execution of sequence: " + sequence.getName() + " (" + 
            std::to_string(sequence.getMoves().size()) + " moves)");
    
    // Start execution by executing the first move
    executeNextMove();
    
    return true; // Execution started successfully (completion handled by response callbacks)
}

void DanceOrchestrator::executeNextMove() {
    if (!execution_active_.load() || !current_sequence_) {
        return;
    }
    
    if (emergency_stop_requested_.load()) {
        logWarn("Emergency stop requested, stopping sequence execution");
        execution_active_.store(false);
        return;
    }
    
    const auto& moves = current_sequence_->getMoves();
    
    // Check if we've completed all moves
    if (current_move_index_ >= moves.size()) {
        logInfo("Sequence execution completed successfully");
        execution_active_.store(false);
        return;
    }
    
    // Execute current move
    const auto& move = moves[current_move_index_];
    logInfo("Executing move " + std::to_string(current_move_index_ + 1) + "/" + 
            std::to_string(moves.size()) + ": " + move.name);
    
    if (!sendMoveRequest(move)) {
        logError("Failed to send move request: " + move.name);
        execution_active_.store(false);
        return;
    }
    
    // Increment move index for next execution
    current_move_index_++;
}

bool DanceOrchestrator::sendMoveRequest(const DanceMove& move) {
    try {
        unitree_api::msg::Request req;
        req.header.identity.api_id = move.api_id;
        req.header.identity.id = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        current_api_id_.store(move.api_id);
        
        logInfo("Sending move request: " + move.name + " (API ID: " + std::to_string(move.api_id) + ")");
        
        // Call appropriate SportClient method based on API ID
        bool success = callSportClientMethod(move, req);
        
        if (!success) {
            logError("Failed to call SportClient method for move: " + move.name);
        }
        
        return success;
        
    } catch (const std::exception& e) {
        logError("Exception while sending move request: " + std::string(e.what()));
        return false;
    }
}


bool DanceOrchestrator::performSafetyCheck() {
    try {
        // Check if robot state client is available
        if (!state_client_) {
            logWarn("Robot state client not available for safety check");
            return true; // Allow execution but warn
        }
        
        // Check robot services status
        std::vector<ServiceState> services;
        if (state_client_->ServiceList(services) != 0) {
            logError("Failed to get service list from robot");
            return false;
        }
        
        // Verify sport mode is active (check for advanced_sport or ai_sport)
        auto sport_service = std::find_if(services.begin(), services.end(),
            [](const ServiceState& s) { 
                return s.name == "advanced_sport" || s.name == "ai_sport" || s.name == "sport"; 
            });
        
        if (sport_service == services.end()) {
            logError("No sport service found in robot services (checked: sport, advanced_sport, ai_sport)");
            return false;
        }
        
        if (sport_service->status != 1) {
            logError("Sport service '" + sport_service->name + "' is not active (status: " + std::to_string(sport_service->status) + ")");
            return false;
        }
        
        logInfo("Safety check passed - sport service is active");
        return true;
        
    } catch (const std::exception& e) {
        logError("Exception during safety check: " + std::string(e.what()));
        return false;
    }
}

void DanceOrchestrator::emergencyStop() {
    logWarn("Emergency stop requested");
    emergency_stop_requested_.store(true);
    execution_active_.store(false);
    
    // Cancel any pending delay timers
    if (delay_timer_) {
        delay_timer_->cancel();
        delay_timer_.reset();
    }
    
    // Send emergency stop command to robot
    try {
        unitree_api::msg::Request req;
        req.header.identity.api_id = 1003; // STOP_MOVE
        req.header.identity.id = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        sport_client_->StopMove(req);
        logInfo("Emergency stop command sent");
        
    } catch (const std::exception& e) {
        logError("Failed to send emergency stop: " + std::string(e.what()));
    }
}

void DanceOrchestrator::scheduleDelayedExecution(float delay_seconds) {
    // Cancel any existing delay timer
    if (delay_timer_) {
        delay_timer_->cancel();
    }
    
    // Create a one-shot timer for the delay
    delay_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delay_seconds * 1000)),
        [this]() {
            if (execution_active_.load() && !emergency_stop_requested_.load()) {
                executeNextMove();
            }
            delay_timer_.reset(); // Clean up the timer
        }
    );
}

void DanceOrchestrator::resetExecutionState() {
    execution_active_.store(false);
    current_api_id_.store(0);
    emergency_stop_requested_.store(false);
    
    // Cancel any pending delay timers
    if (delay_timer_) {
        delay_timer_->cancel();
        delay_timer_.reset();
    }
}

void DanceOrchestrator::logInfo(const std::string& message) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void DanceOrchestrator::logError(const std::string& message) {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

void DanceOrchestrator::logWarn(const std::string& message) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

bool DanceOrchestrator::callSportClientMethod(const DanceMove& move, unitree_api::msg::Request& req) {
    try {
        // Map API IDs to SportClient method calls
        switch (move.api_id) {
            case 1001: // DAMP
                sport_client_->Damp(req);
                break;
            case 1002: // BALANCE_STAND
                sport_client_->BalanceStand(req);
                break;
            case 1003: // STOP_MOVE
                sport_client_->StopMove(req);
                break;
            case 1004: // STAND_UP
                sport_client_->StandUp(req);
                break;
            case 1005: // STAND_DOWN
                sport_client_->StandDown(req);
                break;
            case 1006: // RECOVERY_STAND
                sport_client_->RecoveryStand(req);
                break;
            case 1007: // EULER
                if (move.parameters.contains("roll") && move.parameters.contains("pitch") && move.parameters.contains("yaw")) {
                    sport_client_->Euler(req, 
                        move.parameters["roll"].get<float>(),
                        move.parameters["pitch"].get<float>(),
                        move.parameters["yaw"].get<float>());
                } else {
                    sport_client_->Euler(req, 0.0f, 0.0f, 0.0f);
                }
                break;
            case 1008: // MOVE
                if (move.parameters.contains("vx") && move.parameters.contains("vy") && move.parameters.contains("vyaw")) {
                    sport_client_->Move(req,
                        move.parameters["vx"].get<float>(),
                        move.parameters["vy"].get<float>(),
                        move.parameters["vyaw"].get<float>());
                } else {
                    sport_client_->Move(req, 0.0f, 0.0f, 0.0f);
                }
                break;
            case 1009: // SIT
                sport_client_->Sit(req);
                break;
            case 1010: // RISE_SIT
                sport_client_->RiseSit(req);
                break;
            case 1015: // SPEED_LEVEL
                if (move.parameters.contains("level")) {
                    sport_client_->SpeedLevel(req, move.parameters["level"].get<int>());
                } else {
                    sport_client_->SpeedLevel(req, 1);
                }
                break;
            case 1016: // HELLO
                sport_client_->Hello(req);
                break;
            case 1017: // STRETCH
                sport_client_->Stretch(req);
                break;
            case 1020: // CONTENT
                sport_client_->Content(req);
                break;
            case 1022: // DANCE1
                sport_client_->Dance1(req);
                break;
            case 1023: // DANCE2
                sport_client_->Dance2(req);
                break;
            case 1027: // SWITCH_JOYSTICK
                if (move.parameters.contains("flag")) {
                    sport_client_->SwitchJoystick(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->SwitchJoystick(req, true);
                }
                break;
            case 1028: // POSE
                if (move.parameters.contains("flag")) {
                    sport_client_->Pose(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->Pose(req, true);
                }
                break;
            case 1029: // SCRAPE
                sport_client_->Scrape(req);
                break;
            case 1030: // FRONT_FLIP
                sport_client_->FrontFlip(req);
                break;
            case 1031: // FRONT_JUMP
                sport_client_->FrontJump(req);
                break;
            case 1032: // FRONT_POUNCE
                sport_client_->FrontPounce(req);
                break;
            case 1036: // HEART
                sport_client_->Heart(req);
                break;
            case 1061: // STATIC_WALK
                sport_client_->StaticWalk(req);
                break;
            case 1062: // TROT_RUN
                sport_client_->TrotRun(req);
                break;
            case 1063: // ECONOMIC_GAIT
                sport_client_->EconomicGait(req);
                break;
            case 2041: // LEFT_FLIP
                sport_client_->LeftFlip(req);
                break;
            case 2043: // BACK_FLIP
                sport_client_->BackFlip(req);
                break;
            case 2044: // HANDSTAND
                if (move.parameters.contains("flag")) {
                    sport_client_->HandStand(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->HandStand(req, true);
                }
                break;
            case 2045: // FREE_WALK
                sport_client_->FreeWalk(req);
                break;
            case 2046: // FREE_BOUND
                if (move.parameters.contains("flag")) {
                    sport_client_->FreeBound(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->FreeBound(req, true);
                }
                break;
            case 2047: // FREE_JUMP
                if (move.parameters.contains("flag")) {
                    sport_client_->FreeJump(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->FreeJump(req, true);
                }
                break;
            case 2048: // FREE_AVOID
                if (move.parameters.contains("flag")) {
                    sport_client_->FreeAvoid(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->FreeAvoid(req, true);
                }
                break;
            case 2049: // CLASSIC_WALK
                if (move.parameters.contains("flag")) {
                    sport_client_->ClassicWalk(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->ClassicWalk(req, true);
                }
                break;
            case 2050: // WALK_UPRIGHT
                if (move.parameters.contains("flag")) {
                    sport_client_->WalkUpright(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->WalkUpright(req, true);
                }
                break;
            case 2051: // CROSS_STEP
                if (move.parameters.contains("flag")) {
                    sport_client_->CrossStep(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->CrossStep(req, true);
                }
                break;
            case 2054: // AUTO_RECOVERY_SET
                if (move.parameters.contains("flag")) {
                    sport_client_->AutoRecoverySet(req, move.parameters["flag"].get<bool>());
                } else {
                    sport_client_->AutoRecoverySet(req, true);
                }
                break;
            case 2055: // AUTO_RECOVERY_GET
                {
                    bool flag;
                    sport_client_->AutoRecoveryGet(req, flag);
                    logInfo("Auto recovery status: " + std::string(flag ? "enabled" : "disabled"));
                }
                break;
            case 2058: // SWITCH_AVOID_MODE
                sport_client_->SwitchAvoidMode(req);
                break;
            default:
                logError("Unknown API ID: " + std::to_string(move.api_id));
                return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        logError("Exception in callSportClientMethod: " + std::string(e.what()));
        return false;
    }
}
