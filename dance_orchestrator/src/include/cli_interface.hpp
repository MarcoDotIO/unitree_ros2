#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <sstream>
#include <iostream>
#include "dance_orchestrator.hpp"
#include "dance_sequence.hpp"
#include "move_registry.hpp"

/**
 * @brief Command-line interface for the dance orchestrator
 */
class CLIInterface {
private:
    std::shared_ptr<DanceOrchestrator> orchestrator_;
    std::unique_ptr<MoveRegistry> move_registry_;
    std::unique_ptr<DanceSequence> current_sequence_;
    bool running_;
    std::string config_path_;

public:
    explicit CLIInterface(std::shared_ptr<DanceOrchestrator> orchestrator, const std::string& config_path = "");
    ~CLIInterface() = default;

    // Main interface methods
    void run();
    void runInteractive();
    bool processCommand(const std::string& command_line);
    
    // Direct execution methods (non-interactive)
    bool executeSequenceFile(const std::string& filename);
    bool listMoves();
    bool validateSequenceFile(const std::string& filename);

private:
    // Command handlers
    bool handleCreate(const std::vector<std::string>& args);
    bool handleAdd(const std::vector<std::string>& args);
    bool handleRemove(const std::vector<std::string>& args);
    bool handleList(const std::vector<std::string>& args);
    bool handleListMoves(const std::vector<std::string>& args);
    bool handleSave(const std::vector<std::string>& args);
    bool handleLoad(const std::vector<std::string>& args);
    bool handleExecute(const std::vector<std::string>& args);
    bool handlePreview(const std::vector<std::string>& args);
    bool handleClear(const std::vector<std::string>& args);
    bool handleHelp(const std::vector<std::string>& args);
    bool handleQuit(const std::vector<std::string>& args);

    // Utility methods
    std::vector<std::string> parseCommand(const std::string& command_line);
    std::map<std::string, std::string> parseParameters(const std::vector<std::string>& args, size_t start_index = 1);
    nlohmann::json convertParametersToJson(const std::map<std::string, std::string>& params);
    void printPrompt();
    void printWelcome();
    void printHelp(const std::string& command = "");
    void printError(const std::string& message);
    void printInfo(const std::string& message);
    void printSuccess(const std::string& message);
    
    // Validation methods
    bool validateCurrentSequence();
    bool initializeMoveRegistry();
};
