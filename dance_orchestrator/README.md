# Unitree Go2 Dance Orchestrator

A CLI application for creating and executing dance sequences on the Unitree Go2Edu robot using the ROS2 SDK.

## Features

- **Interactive CLI** - Create, edit, and execute dance sequences
- **39 Built-in Moves** - All Unitree sport actions available
- **Safety Monitoring** - Checks robot status before execution
- **Response-based Completion** - Waits for move completion via `/api/sport/response`
- **Sequence Management** - Save/load sequences to/from JSON files
- **Emergency Stop** - Immediate stop functionality during execution
- **Parameter Support** - Customize moves with parameters (velocity, angles, etc.)

## Prerequisites

- ROS2 Humble
- Unitree Go2Edu robot
- Unitree ROS2 SDK packages (`unitree_api`, `unitree_go`)
- Robot connected and sport mode active

## Building

```bash
cd /home/marcodotio/Projects/ROS/unitree_ros2/dance_orchestrator
colcon build
source install/setup.bash
```

## Usage

### Starting the Application

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the dance orchestrator
./install/dance_orchestrator/lib/dance_orchestrator/dance_orchestrator
```

### Basic Commands

| Command | Description | Example |
|---------|-------------|---------|
| `help` | Show all commands | `help` |
| `list-moves` | Show available moves | `list-moves` |
| `create <name>` | Create new sequence | `create my_dance` |
| `add <move> [params]` | Add move to sequence | `add move vx=0.3 vy=0.0` |
| `list` | Show current sequence | `list` |
| `execute` | Execute current sequence | `execute` |
| `save <file>` | Save sequence | `save my_dance.json` |
| `load <file>` | Load sequence | `load my_dance.json` |
| `clear` | Clear current sequence | `clear` |
| `quit` | Exit application | `quit` |

### Available Moves

The system includes 39 built-in moves:

**Basic Movements:**
- `stand_up`, `stand_down`, `sit`, `rise_sit`
- `move` (with vx, vy, vyaw parameters)
- `euler` (with roll, pitch, yaw parameters)

**Dance Moves:**
- `dance1`, `dance2`, `hello`, `stretch`, `content`

**Advanced Moves:**
- `front_flip`, `back_flip`, `left_flip`
- `front_jump`, `front_pounce`
- `hand_stand`, `scrape`, `heart`

**Gait Modes:**
- `static_walk`, `trot_run`, `economic_gait`
- `free_walk`, `classic_walk`, `walk_upright`

### Creating Dance Sequences

1. **Start a new sequence:**
   ```
   dance> create my_first_dance
   ```

2. **Add moves with parameters:**
   ```
   dance> add stand_up
   dance> add move vx=0.3 vy=0.0 vyaw=0.0
   dance> add euler roll=0.0 pitch=0.2 yaw=0.0
   dance> add dance1
   dance> add sit
   ```

3. **Preview the sequence:**
   ```
   dance> list
   ```

4. **Save for later use:**
   ```
   dance> save my_first_dance.json
   ```

5. **Execute the sequence:**
   ```
   dance> execute
   ```

### Parameter Examples

**Movement with velocity:**
```
add move vx=0.5 vy=0.2 vyaw=0.3
```

**Euler angles (radians):**
```
add euler roll=0.1 pitch=0.2 yaw=0.0
```

**Speed level:**
```
add speed_level level=2
```

**Boolean parameters:**
```
add pose enable=true
add joystick enable=false
```

### Safety Features

- **Pre-execution checks** - Verifies robot is in sport mode
- **Emergency stop** - Press Ctrl+C during execution
- **Timeout protection** - Each move has configurable timeout
- **Error recovery** - Automatic stop on failures

### File Management

Sequences are saved as JSON files in the current directory:

```json
{
  "name": "my_dance",
  "moves": [
    {
      "name": "stand_up",
      "api_id": 1001,
      "timeout_seconds": 3.0,
      "parameters": {}
    },
    {
      "name": "move",
      "api_id": 1008,
      "timeout_seconds": 2.0,
      "parameters": {
        "vx": 0.3,
        "vy": 0.0,
        "vyaw": 0.0
      }
    }
  ]
}
```

### Troubleshooting

**Robot not responding:**
- Ensure robot is powered on and connected
- Check that sport mode is active
- Verify ROS2 topics are available: `ros2 topic list`

**Build errors:**
- Make sure all dependencies are installed
- Source ROS2 setup: `source /opt/ros/humble/setup.bash`
- Clean build: `rm -rf build install log && colcon build`

**Execution timeouts:**
- Check robot battery level
- Ensure stable network connection
- Verify no other applications are controlling the robot

### Example Session

```
$ ./install/dance_orchestrator/lib/dance_orchestrator/dance_orchestrator

=== Unitree Go2 Dance Orchestrator ===
Type 'help' for available commands.

dance> create hello_world
Info: Created new sequence: hello_world

dance> add stand_up
Info: Added move: stand_up

dance> add hello
Info: Added move: hello

dance> add sit
Info: Added move: sit

dance> list
Current sequence: hello_world
1. stand_up (timeout: 3.0s)
2. hello (timeout: 5.0s)  
3. sit (timeout: 3.0s)

dance> execute
Info: Starting sequence execution: hello_world
Info: Executing move 1/3: stand_up
Info: Move completed: stand_up
Info: Executing move 2/3: hello
Info: Move completed: hello
Info: Executing move 3/3: sit
Info: Move completed: sit
Info: Sequence execution completed successfully

dance> save hello_world.json
Info: Sequence saved to: hello_world.json

dance> quit
Info: Goodbye!
```

## Configuration

Move configurations are stored in `config/moves.json`. You can modify timeouts, parameters, and safety levels as needed.

## Development

The project structure:
- `src/main.cpp` - Application entry point
- `src/dance_orchestrator.cpp` - Core execution engine
- `src/cli_interface.cpp` - Command-line interface
- `src/move_registry.cpp` - Move definitions and loading
- `include/` - Header files
- `config/moves.json` - Move configuration

## License

This project follows the same license as the Unitree ROS2 SDK.
