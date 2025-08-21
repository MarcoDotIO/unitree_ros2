#include "cli_interface.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>

CLIInterface::CLIInterface(std::shared_ptr<DanceOrchestrator> orchestrator, const std::string& config_path)
    : orchestrator_(orchestrator), running_(false), config_path_(config_path) {
    
    move_registry_ = std::make_unique<MoveRegistry>();
    
    // Set default config path if not provided
    if (config_path_.empty()) {
        config_path_ = "config/moves.json";
    }
    
    initializeMoveRegistry();
}

bool CLIInterface::initializeMoveRegistry() {
    if (!move_registry_->loadFromFile(config_path_)) {
        printError("Failed to load move registry from: " + config_path_);
        return false;
    }
    
    printInfo("Loaded " + std::to_string(move_registry_->size()) + " moves from registry");
    return true;
}

void CLIInterface::run() {
    runInteractive();
}

void CLIInterface::runInteractive() {
    printWelcome();
    running_ = true;
    
    std::string input;
    while (running_) {
        printPrompt();
        
        if (!std::getline(std::cin, input)) {
            // EOF or error
            break;
        }
        
        // Trim whitespace
        input.erase(0, input.find_first_not_of(" \t"));
        input.erase(input.find_last_not_of(" \t") + 1);
        
        if (input.empty()) {
            continue;
        }
        
        processCommand(input);
    }
    
    printInfo("Goodbye!");
}

bool CLIInterface::processCommand(const std::string& command_line) {
    auto args = parseCommand(command_line);
    if (args.empty()) {
        return false;
    }
    
    std::string command = args[0];
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);
    
    try {
        if (command == "create") {
            return handleCreate(args);
        } else if (command == "add") {
            return handleAdd(args);
        } else if (command == "remove" || command == "rm") {
            return handleRemove(args);
        } else if (command == "list" || command == "ls") {
            return handleList(args);
        } else if (command == "list-moves" || command == "moves") {
            return handleListMoves(args);
        } else if (command == "save") {
            return handleSave(args);
        } else if (command == "load") {
            return handleLoad(args);
        } else if (command == "execute" || command == "exec") {
            return handleExecute(args);
        } else if (command == "preview") {
            return handlePreview(args);
        } else if (command == "clear") {
            return handleClear(args);
        } else if (command == "help" || command == "h" || command == "?") {
            return handleHelp(args);
        } else if (command == "quit" || command == "exit" || command == "q") {
            return handleQuit(args);
        } else {
            printError("Unknown command: " + command + ". Type 'help' for available commands.");
            return false;
        }
    } catch (const std::exception& e) {
        printError("Command failed: " + std::string(e.what()));
        return false;
    }
}

std::vector<std::string> CLIInterface::parseCommand(const std::string& command_line) {
    std::vector<std::string> tokens;
    std::istringstream iss(command_line);
    std::string token;
    
    while (iss >> token) {
        tokens.push_back(token);
    }
    
    return tokens;
}

std::map<std::string, std::string> CLIInterface::parseParameters(const std::vector<std::string>& args, size_t start_index) {
    std::map<std::string, std::string> params;
    
    for (size_t i = start_index; i < args.size(); ++i) {
        const std::string& arg = args[i];
        
        // Look for key=value format
        size_t eq_pos = arg.find('=');
        if (eq_pos != std::string::npos) {
            std::string key = arg.substr(0, eq_pos);
            std::string value = arg.substr(eq_pos + 1);
            params[key] = value;
        }
    }
    
    return params;
}

nlohmann::json CLIInterface::convertParametersToJson(const std::map<std::string, std::string>& params) {
    nlohmann::json json_params;
    
    for (const auto& [key, value] : params) {
        // Try to parse as different types
        if (value == "true") {
            json_params[key] = true;
        } else if (value == "false") {
            json_params[key] = false;
        } else {
            // Try to parse as number
            try {
                if (value.find('.') != std::string::npos) {
                    json_params[key] = std::stof(value);
                } else {
                    json_params[key] = std::stoi(value);
                }
            } catch (...) {
                // Keep as string
                json_params[key] = value;
            }
        }
    }
    
    return json_params;
}

bool CLIInterface::handleCreate(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        printError("Usage: create <sequence_name>");
        return false;
    }
    
    std::string sequence_name = args[1];
    current_sequence_ = std::make_unique<DanceSequence>(sequence_name);
    
    printSuccess("Created new sequence: " + sequence_name);
    return true;
}

bool CLIInterface::handleAdd(const std::vector<std::string>& args) {
    if (!current_sequence_) {
        printError("No sequence created. Use 'create <name>' first.");
        return false;
    }
    
    if (args.size() < 2) {
        printError("Usage: add <move> [param=value ...] OR add delay(<seconds>)");
        return false;
    }
    
    std::string item_name = args[1];
    
    // Check if this is a delay command
    if (item_name.substr(0, 5) == "delay" && item_name.find('(') != std::string::npos) {
        // Parse delay(X.X) format
        size_t start = item_name.find('(');
        size_t end = item_name.find(')', start);
        
        if (start == std::string::npos || end == std::string::npos) {
            printError("Invalid delay format. Use: add delay(1.0)");
            return false;
        }
        
        std::string delay_str = item_name.substr(start + 1, end - start - 1);
        float delay_seconds;
        
        try {
            delay_seconds = std::stof(delay_str);
        } catch (const std::exception& e) {
            printError("Invalid delay value: " + delay_str);
            return false;
        }
        
        // Add delay to the last move in sequence
        if (current_sequence_->empty()) {
            printError("Cannot add delay to empty sequence. Add a move first.");
            return false;
        }
        
        // Get the last move and modify its delay
        auto moves = current_sequence_->getMoves();
        auto& last_move = const_cast<DanceMove&>(moves.back());
        last_move.delay_after = delay_seconds;
        
        // Remove and re-add the move to update the sequence
        current_sequence_->removeMove(current_sequence_->size() - 1);
        current_sequence_->addMove(last_move);
        
        printSuccess("Added " + std::to_string(delay_seconds) + " second delay after last move");
        return true;
    }
    
    // Handle regular move addition
    if (!move_registry_->hasMove(item_name)) {
        printError("Unknown move: " + item_name + ". Use 'list-moves' to see available moves.");
        return false;
    }
    
    // Parse parameters
    auto params = parseParameters(args, 2);
    auto json_params = convertParametersToJson(params);
    
    // Create and validate move
    try {
        DanceMove move = move_registry_->createMove(item_name, json_params);
        current_sequence_->addMove(move);
        
        printSuccess("Added move: " + item_name + " to sequence");
        if (!params.empty()) {
            printInfo("Parameters: " + json_params.dump());
        }
        
    } catch (const std::exception& e) {
        printError("Failed to add move: " + std::string(e.what()));
        return false;
    }
    
    return true;
}

bool CLIInterface::handleRemove(const std::vector<std::string>& args) {
    if (!current_sequence_) {
        printError("No sequence created. Use 'create <name>' first.");
        return false;
    }
    
    if (args.size() < 2) {
        printError("Usage: remove <index>");
        return false;
    }
    
    try {
        size_t index = std::stoul(args[1]);
        if (index == 0 || index > current_sequence_->size()) {
            printError("Invalid index. Use 'list' to see current sequence.");
            return false;
        }
        
        // Convert to 0-based index
        index--;
        
        std::string move_name = current_sequence_->getMove(index).name;
        current_sequence_->removeMove(index);
        
        printSuccess("Removed move: " + move_name);
        
    } catch (const std::exception& e) {
        printError("Invalid index: " + args[1]);
        return false;
    }
    
    return true;
}

bool CLIInterface::handleList(const std::vector<std::string>& args) {
    if (!current_sequence_) {
        printInfo("No sequence created.");
        return true;
    }
    
    if (current_sequence_->empty()) {
        printInfo("Current sequence is empty.");
        return true;
    }
    
    std::cout << "\nCurrent sequence: " << current_sequence_->getName() << std::endl;
    std::cout << "Inter-move delay: " << current_sequence_->getInterMoveDelay() << "s" << std::endl;
    std::cout << "Estimated duration: " << current_sequence_->getTotalEstimatedDuration() << "s" << std::endl;
    std::cout << "\nMoves:" << std::endl;
    
    const auto& moves = current_sequence_->getMoves();
    for (size_t i = 0; i < moves.size(); ++i) {
        const auto& move = moves[i];
        std::cout << "  " << (i + 1) << ". " << move.name 
                  << " (API " << move.api_id << ", " << move.timeout_seconds << "s)";
        
        if (!move.parameters.empty()) {
            std::cout << " - " << move.parameters.dump();
        }
        
        std::cout << std::endl;
    }
    std::cout << std::endl;
    
    return true;
}

bool CLIInterface::handleListMoves(const std::vector<std::string>& args) {
    auto available_moves = move_registry_->getAvailableMoves();
    
    if (available_moves.empty()) {
        printInfo("No moves available in registry.");
        return true;
    }
    
    std::cout << "\nAvailable moves (" << available_moves.size() << "):" << std::endl;
    
    for (const auto& move_name : available_moves) {
        std::cout << "  " << move_registry_->getMoveInfo(move_name) << std::endl;
    }
    std::cout << std::endl;
    
    return true;
}

bool CLIInterface::handleSave(const std::vector<std::string>& args) {
    if (!current_sequence_) {
        printError("No sequence to save. Use 'create <name>' first.");
        return false;
    }
    
    if (args.size() < 2) {
        printError("Usage: save <filename>");
        return false;
    }
    
    std::string filename = args[1];
    
    // Add .json extension if not present
    if (filename.find('.') == std::string::npos) {
        filename += ".json";
    }
    
    try {
        current_sequence_->saveToFile(filename);
        printSuccess("Sequence saved to: " + filename);
        
    } catch (const std::exception& e) {
        printError("Failed to save sequence: " + std::string(e.what()));
        return false;
    }
    
    return true;
}

bool CLIInterface::handleLoad(const std::vector<std::string>& args) {
    if (args.size() < 2) {
        printError("Usage: load <filename>");
        return false;
    }
    
    std::string filename = args[1];
    
    // Add .json extension if not present
    if (filename.find('.') == std::string::npos) {
        filename += ".json";
    }
    
    try {
        auto loaded_sequence = std::make_unique<DanceSequence>(DanceSequence::loadFromFile(filename));
        current_sequence_ = std::move(loaded_sequence);
        
        printSuccess("Loaded sequence: " + current_sequence_->getName() + " from " + filename);
        printInfo("Sequence has " + std::to_string(current_sequence_->size()) + " moves");
        
    } catch (const std::exception& e) {
        printError("Failed to load sequence: " + std::string(e.what()));
        return false;
    }
    
    return true;
}

bool CLIInterface::handleExecute(const std::vector<std::string>& args) {
    DanceSequence* sequence_to_execute = nullptr;
    std::unique_ptr<DanceSequence> temp_sequence;
    
    if (args.size() >= 2) {
        // Execute from file
        std::string filename = args[1];
        if (filename.find('.') == std::string::npos) {
            filename += ".json";
        }
        
        try {
            temp_sequence = std::make_unique<DanceSequence>(DanceSequence::loadFromFile(filename));
            sequence_to_execute = temp_sequence.get();
            
        } catch (const std::exception& e) {
            printError("Failed to load sequence file: " + std::string(e.what()));
            return false;
        }
    } else {
        // Execute current sequence
        if (!current_sequence_) {
            printError("No sequence to execute. Use 'create <name>' and 'add <move>' first, or 'execute <filename>'.");
            return false;
        }
        sequence_to_execute = current_sequence_.get();
    }
    
    if (sequence_to_execute->empty()) {
        printError("Cannot execute empty sequence.");
        return false;
    }
    
    printInfo("Executing sequence: " + sequence_to_execute->getName());
    printInfo("This will take approximately " + std::to_string(sequence_to_execute->getTotalEstimatedDuration()) + " seconds");
    
    try {
        bool success = orchestrator_->executeSequence(*sequence_to_execute);
        
        if (success) {
            printSuccess("Sequence executed successfully!");
        } else {
            printError("Sequence execution failed or was interrupted.");
        }
        
        return success;
        
    } catch (const std::exception& e) {
        printError("Execution error: " + std::string(e.what()));
        return false;
    }
}

bool CLIInterface::handlePreview(const std::vector<std::string>& args) {
    DanceSequence* sequence_to_preview = nullptr;
    std::unique_ptr<DanceSequence> temp_sequence;
    
    if (args.size() >= 2) {
        // Preview from file
        std::string filename = args[1];
        if (filename.find('.') == std::string::npos) {
            filename += ".json";
        }
        
        try {
            temp_sequence = std::make_unique<DanceSequence>(DanceSequence::loadFromFile(filename));
            sequence_to_preview = temp_sequence.get();
            
        } catch (const std::exception& e) {
            printError("Failed to load sequence file: " + std::string(e.what()));
            return false;
        }
    } else {
        // Preview current sequence
        if (!current_sequence_) {
            printError("No sequence to preview. Use 'create <name>' and 'add <move>' first, or 'preview <filename>'.");
            return false;
        }
        sequence_to_preview = current_sequence_.get();
    }
    
    std::cout << "\n=== SEQUENCE PREVIEW ===" << std::endl;
    std::cout << "Name: " << sequence_to_preview->getName() << std::endl;
    std::cout << "Moves: " << sequence_to_preview->size() << std::endl;
    std::cout << "Inter-move delay: " << sequence_to_preview->getInterMoveDelay() << "s" << std::endl;
    std::cout << "Estimated duration: " << sequence_to_preview->getTotalEstimatedDuration() << "s" << std::endl;
    
    if (!sequence_to_preview->empty()) {
        std::cout << "\nExecution plan:" << std::endl;
        const auto& moves = sequence_to_preview->getMoves();
        float cumulative_time = 0.0f;
        
        for (size_t i = 0; i < moves.size(); ++i) {
            const auto& move = moves[i];
            std::cout << "  " << std::to_string(cumulative_time) << "s: " 
                      << move.name << " (" << move.timeout_seconds << "s)";
            
            if (!move.parameters.empty()) {
                std::cout << " - " << move.parameters.dump();
            }
            
            std::cout << std::endl;
            
            cumulative_time += move.timeout_seconds;
            if (i < moves.size() - 1) {
                cumulative_time += sequence_to_preview->getInterMoveDelay();
            }
        }
    }
    
    std::cout << "========================\n" << std::endl;
    
    return true;
}

bool CLIInterface::handleClear(const std::vector<std::string>& args) {
    if (!current_sequence_) {
        printInfo("No sequence to clear.");
        return true;
    }
    
    std::string sequence_name = current_sequence_->getName();
    current_sequence_->clear();
    
    printSuccess("Cleared all moves from sequence: " + sequence_name);
    return true;
}

bool CLIInterface::handleHelp(const std::vector<std::string>& args) {
    if (args.size() >= 2) {
        printHelp(args[1]);
    } else {
        printHelp();
    }
    return true;
}

bool CLIInterface::handleQuit(const std::vector<std::string>& args) {
    running_ = false;
    return true;
}

void CLIInterface::printPrompt() {
    std::string sequence_info = "";
    if (current_sequence_) {
        sequence_info = " [" + current_sequence_->getName() + ":" + std::to_string(current_sequence_->size()) + "]";
    }
    
    std::cout << "dance" << sequence_info << "> ";
}

void CLIInterface::printWelcome() {
    std::cout << "\n=== Unitree Go2 Dance Orchestrator ===" << std::endl;
    std::cout << "Type 'help' for available commands." << std::endl;
    std::cout << "Type 'quit' to exit.\n" << std::endl;
}

void CLIInterface::printHelp(const std::string& command) {
    if (command.empty()) {
        std::cout << "\nAvailable commands:" << std::endl;
        std::cout << "  create <name>           - Create a new dance sequence" << std::endl;
        std::cout << "  add <move> [params...]  - Add move to current sequence" << std::endl;
        std::cout << "  remove <index>          - Remove move by index (1-based)" << std::endl;
        std::cout << "  list                    - Show current sequence" << std::endl;
        std::cout << "  list-moves              - Show available moves" << std::endl;
        std::cout << "  save <filename>         - Save current sequence" << std::endl;
        std::cout << "  load <filename>         - Load sequence from file" << std::endl;
        std::cout << "  execute [filename]      - Execute sequence" << std::endl;
        std::cout << "  preview [filename]      - Preview sequence without executing" << std::endl;
        std::cout << "  clear                   - Clear current sequence" << std::endl;
        std::cout << "  help [command]          - Show help" << std::endl;
        std::cout << "  quit                    - Exit program" << std::endl;
        std::cout << "\nParameter format: key=value (e.g., vx=0.3 vy=0.0)" << std::endl;
        std::cout << "Example: add move vx=0.3 vy=0.0 vyaw=0.5\n" << std::endl;
    } else {
        // Command-specific help would go here
        std::cout << "Help for command: " << command << std::endl;
        std::cout << "Use 'help' for general command list." << std::endl;
    }
}

void CLIInterface::printError(const std::string& message) {
    std::cout << "Error: " << message << std::endl;
}

void CLIInterface::printInfo(const std::string& message) {
    std::cout << "Info: " << message << std::endl;
}

void CLIInterface::printSuccess(const std::string& message) {
    std::cout << "Success: " << message << std::endl;
}

bool CLIInterface::executeSequenceFile(const std::string& filename) {
    return processCommand("execute " + filename);
}

bool CLIInterface::listMoves() {
    return processCommand("list-moves");
}

bool CLIInterface::validateSequenceFile(const std::string& filename) {
    try {
        DanceSequence::loadFromFile(filename);
        printSuccess("Sequence file is valid: " + filename);
        return true;
    } catch (const std::exception& e) {
        printError("Invalid sequence file: " + std::string(e.what()));
        return false;
    }
}
