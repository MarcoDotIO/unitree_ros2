# Dance Orchestrator Implementation Specification

## Overview

A CLI application for creating and executing dance sequences on the Unitree Go2Edu robot using the existing ROS2 SDK. The orchestrator leverages response-based completion detection via `/api/sport/response` to enable precise choreography control.

## Phase 1: Core Infrastructure

### 1.1 Project Structure
```
dance_orchestrator/
├── include/
│   ├── dance_orchestrator.hpp
│   ├── dance_sequence.hpp
│   ├── dance_move.hpp
│   └── move_registry.hpp
├── src/
│   ├── dance_orchestrator.cpp
│   ├── dance_sequence.cpp
│   ├── move_registry.cpp
│   └── main.cpp
├── config/
│   ├── sequences/           # Saved dance sequences
│   └── moves.json          # Move definitions
├── CMakeLists.txt
└── package.xml
```

### 1.2 Core Classes

**DanceMove Class:**
```cpp
struct DanceMove {
    std::string name;           // "dance1", "front_flip", etc.
    int32_t api_id;            // Sport API ID
    nlohmann::json parameters; // Move-specific params
    float timeout_seconds;     // Max execution time
    std::string description;   // Human-readable description
};
```

**DanceSequence Class:**
```cpp
class DanceSequence {
    std::string name_;
    std::vector<DanceMove> moves_;
    float inter_move_delay_;   // Pause between moves
    
public:
    void addMove(const DanceMove& move);
    void removeMove(size_t index);
    void saveToFile(const std::string& filepath);
    static DanceSequence loadFromFile(const std::string& filepath);
    nlohmann::json toJson() const;
    void fromJson(const nlohmann::json& json);
};
```

**DanceOrchestrator Class:**
```cpp
class DanceOrchestrator : public rclcpp::Node {
    SportClient sport_client_;
    RobotStateClient state_client_;
    std::unique_ptr<DanceSequence> current_sequence_;
    
    // Response monitoring
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
    std::promise<bool> move_completion_promise_;
    std::atomic<bool> execution_active_{false};
    
public:
    bool executeSequence(const DanceSequence& sequence);
    bool executeMove(const DanceMove& move);
    void emergencyStop();
    bool waitForMoveCompletion(int32_t api_id, float timeout);
};
```

## Phase 2: Move Registry System

### 2.1 Move Definitions
Create `moves.json` configuration:
```json
{
  "moves": {
    "dance1": {
      "api_id": 1022,
      "timeout": 15.0,
      "description": "Built-in dance routine 1",
      "parameters": {},
      "safety_level": "safe"
    },
    "front_flip": {
      "api_id": 1030,
      "timeout": 5.0,
      "description": "Forward flip",
      "parameters": {},
      "safety_level": "advanced"
    },
    "move": {
      "api_id": 1008,
      "timeout": 2.0,
      "description": "Move with velocity",
      "parameters": {
        "vx": {"type": "float", "default": 0.0, "range": [-1.0, 1.0]},
        "vy": {"type": "float", "default": 0.0, "range": [-1.0, 1.0]},
        "vyaw": {"type": "float", "default": 0.0, "range": [-2.0, 2.0]}
      },
      "safety_level": "safe"
    }
  }
}
```

### 2.2 MoveRegistry Class
```cpp
class MoveRegistry {
    std::map<std::string, DanceMove> registered_moves_;
    
public:
    bool loadFromFile(const std::string& config_file);
    std::vector<std::string> getAvailableMoves() const;
    DanceMove getMove(const std::string& name) const;
    bool validateParameters(const std::string& move_name, const nlohmann::json& params);
};
```

## Phase 3: CLI Interface

### 3.1 Command Structure
```bash
# Interactive mode
./dance_orchestrator

# Direct execution
./dance_orchestrator execute sequence.json
./dance_orchestrator list-moves
./dance_orchestrator validate sequence.json
```

### 3.2 Interactive Commands
```
> create <sequence_name>     # Start new sequence
> add <move> [params...]     # Add move to current sequence
> remove <index>             # Remove move by index
> list                       # Show current sequence
> list-moves                 # Show available moves
> save <filename>            # Save current sequence
> load <filename>            # Load sequence from file
> execute [filename]         # Execute sequence
> preview [filename]         # Show sequence without executing
> clear                      # Clear current sequence
> help [command]             # Show help
> quit                       # Exit
```

### 3.3 Parameter Handling
```bash
# Examples of parameterized moves
> add move vx=0.3 vy=0.0 vyaw=0.5
> add euler roll=0.1 pitch=0.0 yaw=0.2
> add speed_level level=2
```

## Phase 4: Execution Engine

### 4.1 Response-Based Completion Detection
```cpp
void DanceOrchestrator::setupResponseMonitoring() {
    response_sub_ = create_subscription<unitree_api::msg::Response>(
        "/api/sport/response", 10,
        [this](const unitree_api::msg::Response::SharedPtr msg) {
            handleSportResponse(msg);
        });
}

void DanceOrchestrator::handleSportResponse(const unitree_api::msg::Response::SharedPtr msg) {
    if (execution_active_ && msg->header.identity.api_id == current_api_id_) {
        if (msg->header.status.code == 0) {
            move_completion_promise_.set_value(true);  // Success
        } else {
            RCLCPP_ERROR(get_logger(), "Move failed with code: %d", msg->header.status.code);
            move_completion_promise_.set_value(false); // Failure
        }
    }
}
```

### 4.2 Sequence Execution Logic
```cpp
bool DanceOrchestrator::executeSequence(const DanceSequence& sequence) {
    for (const auto& move : sequence.getMoves()) {
        // Pre-move safety check
        if (!performSafetyCheck()) {
            RCLCPP_ERROR(get_logger(), "Safety check failed, aborting sequence");
            return false;
        }
        
        // Execute move
        if (!executeMove(move)) {
            RCLCPP_ERROR(get_logger(), "Move '%s' failed, stopping sequence", move.name.c_str());
            return false;
        }
        
        // Inter-move delay
        if (sequence.getInterMoveDelay() > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(sequence.getInterMoveDelay() * 1000))
            );
        }
    }
    return true;
}
```

## Phase 5: Safety & Error Handling

### 5.1 Safety Checks
```cpp
bool DanceOrchestrator::performSafetyCheck() {
    // Check robot state
    std::vector<ServiceState> services;
    if (state_client_.ServiceList(services) != 0) {
        return false;
    }
    
    // Verify sport mode is active
    auto sport_service = std::find_if(services.begin(), services.end(),
        [](const ServiceState& s) { return s.name == "sport"; });
    
    return sport_service != services.end() && sport_service->status == 1;
}
```

### 5.2 Error Recovery
```cpp
enum class ExecutionError {
    TIMEOUT,
    ROBOT_ERROR,
    SAFETY_VIOLATION,
    COMMUNICATION_FAILURE
};

class ErrorHandler {
public:
    bool handleError(ExecutionError error, const DanceMove& failed_move);
    void emergencyStop();
    bool attemptRecovery();
};
```

## Phase 6: File Format & Persistence

### 6.1 Sequence File Format (JSON)
```json
{
  "name": "my_dance_routine",
  "version": "1.0",
  "created": "2025-08-20T22:11:00Z",
  "inter_move_delay": 1.0,
  "moves": [
    {
      "name": "stand_up",
      "api_id": 1004,
      "parameters": {},
      "timeout": 3.0
    },
    {
      "name": "dance1",
      "api_id": 1022,
      "parameters": {},
      "timeout": 15.0
    },
    {
      "name": "move",
      "api_id": 1008,
      "parameters": {
        "vx": 0.3,
        "vy": 0.0,
        "vyaw": 0.5
      },
      "timeout": 2.0
    }
  ]
}
```

## Phase 7: Build Integration

### 7.1 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(dance_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(dance_orchestrator
  src/main.cpp
  src/dance_orchestrator.cpp
  src/dance_sequence.cpp
  src/move_registry.cpp
)

target_include_directories(dance_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dance_orchestrator
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/ros2_sport_client.cpp
)

ament_target_dependencies(dance_orchestrator
  rclcpp
  unitree_api
  unitree_go
  nlohmann_json
)

install(TARGETS dance_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
```

## Available Sport Actions

Based on the Unitree SDK analysis, the following sport actions are available:

### Basic Motions (1000s)
- `DAMP` (1001) - Damping mode
- `BALANCESTAND` (1002) - Balance standing
- `STOPMOVE` (1003) - Stop movement
- `STANDUP` (1004) - Stand up
- `STANDDOWN` (1005) - Stand down
- `RECOVERYSTAND` (1006) - Recovery stand
- `EULER` (1007) - Euler angle control
- `MOVE` (1008) - Velocity movement
- `SIT` (1009) - Sit down
- `RISESIT` (1010) - Rise from sit

### Dance & Gestures
- `DANCE1` (1022) - Built-in dance routine 1
- `DANCE2` (1023) - Built-in dance routine 2
- `HELLO` (1016) - Greeting gesture
- `STRETCH` (1017) - Stretching motion
- `HEART` (1036) - Heart gesture
- `SCRAPE` (1029) - Scraping motion

### Advanced Acrobatics (2000s)
- `FRONTFLIP` (1030) - Forward flip
- `BACKFLIP` (2043) - Backward flip
- `LEFTFLIP` (2041) - Left flip
- `FRONTJUMP` (1031) - Forward jump
- `HANDSTAND` (2044) - Handstand
- `FRONTPOUNCE` (1032) - Forward pounce

### Locomotion Modes
- `STATICWALK` (1061) - Static walk
- `TROTRUN` (1062) - Trot run
- `ECONOMICGAIT` (1063) - Economic gait
- `FREEWALK` (2045) - Free walk
- `CLASSICWALK` (2049) - Classic walk
- `WALKUPRIGHT` (2050) - Walk upright
- `CROSSSTEP` (2051) - Cross step

## Implementation Order

1. **Phase 1**: Core classes and basic structure
2. **Phase 2**: Move registry and configuration system
3. **Phase 3**: CLI interface and command parsing
4. **Phase 4**: Response monitoring and execution engine
5. **Phase 5**: Safety systems and error handling
6. **Phase 6**: File persistence and sequence management
7. **Phase 7**: Build system integration and testing

Each phase builds incrementally, allowing for testing and validation at each step.

## Key Technical Features

- **Response-Based Completion**: Uses `/api/sport/response` endpoint to detect when moves complete
- **Safety Monitoring**: Integrates with `RobotStateClient` for safety checks
- **Parameterized Moves**: Supports moves with parameters (velocity, angles, etc.)
- **Sequence Persistence**: JSON-based save/load functionality
- **Error Recovery**: Comprehensive error handling and recovery mechanisms
- **Interactive CLI**: User-friendly command-line interface for sequence creation
