#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <thread>
#include <filesystem>
#include "dance_orchestrator.hpp"
#include "cli_interface.hpp"

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  (no args)                 - Interactive mode" << std::endl;
    std::cout << "  execute <sequence.json>   - Execute sequence file" << std::endl;
    std::cout << "  list-moves               - List available moves" << std::endl;
    std::cout << "  validate <sequence.json> - Validate sequence file" << std::endl;
}

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    try {
        // Create the dance orchestrator node
        auto orchestrator = std::make_shared<DanceOrchestrator>("dance_orchestrator");
        
        // Create CLI interface with config path
        std::string config_path = "config/moves.json";
        
        // Check if config exists, try alternative paths
        if (!std::filesystem::exists(config_path)) {
            config_path = "../config/moves.json";
            if (!std::filesystem::exists(config_path)) {
                config_path = "dance_orchestrator/config/moves.json";
            }
        }
        
        auto cli = std::make_unique<CLIInterface>(orchestrator, config_path);
        
        // Start ROS2 spinning in background thread
        std::thread ros_thread([&orchestrator]() {
            rclcpp::spin(orchestrator);
        });
        
        // Handle command line arguments
        if (argc == 1) {
            // Interactive mode
            cli->run();
        } else if (argc >= 2) {
            std::string command = argv[1];
            
            if (command == "execute" && argc >= 3) {
                std::string filename = argv[2];
                bool success = cli->executeSequenceFile(filename);
                
                // Cleanup
                rclcpp::shutdown();
                ros_thread.join();
                return success ? 0 : 1;
                
            } else if (command == "list-moves") {
                bool success = cli->listMoves();
                
                // Cleanup
                rclcpp::shutdown();
                ros_thread.join();
                return success ? 0 : 1;
                
            } else if (command == "validate" && argc >= 3) {
                std::string filename = argv[2];
                bool success = cli->validateSequenceFile(filename);
                
                // Cleanup
                rclcpp::shutdown();
                ros_thread.join();
                return success ? 0 : 1;
                
            } else {
                printUsage(argv[0]);
                
                // Cleanup
                rclcpp::shutdown();
                ros_thread.join();
                return 1;
            }
        }
        
        // Cleanup for interactive mode
        rclcpp::shutdown();
        ros_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    return 0;
}
