#pragma once

#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include "dance_move.hpp"
#include "nlohmann/json.hpp"

/**
 * @brief Registry for managing available dance moves and their configurations
 */
class MoveRegistry {
private:
    std::map<std::string, DanceMove> registered_moves_;
    std::string config_file_path_;

public:
    MoveRegistry() = default;
    explicit MoveRegistry(const std::string& config_file) : config_file_path_(config_file) {}

    /**
     * @brief Load move definitions from JSON configuration file
     * @param config_file Path to the moves.json file
     * @return true if successful, false otherwise
     */
    bool loadFromFile(const std::string& config_file) {
        try {
            std::ifstream file(config_file);
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open config file: " + config_file);
            }

            nlohmann::json config;
            file >> config;
            file.close();

            if (!config.contains("moves") || !config["moves"].is_object()) {
                throw std::runtime_error("Invalid config format: missing 'moves' object");
            }

            registered_moves_.clear();
            config_file_path_ = config_file;

            for (const auto& [move_name, move_config] : config["moves"].items()) {
                DanceMove move;
                move.name = move_name;
                move.api_id = move_config.at("api_id").get<int32_t>();
                move.timeout_seconds = move_config.at("timeout").get<float>();
                move.description = move_config.value("description", "");
                move.parameters = move_config.value("parameters", nlohmann::json{});

                registered_moves_[move_name] = move;
            }

            return true;

        } catch (const std::exception& e) {
            // Log error would go here in production
            return false;
        }
    }

    /**
     * @brief Get list of all available move names
     * @return Vector of move names
     */
    std::vector<std::string> getAvailableMoves() const {
        std::vector<std::string> move_names;
        move_names.reserve(registered_moves_.size());
        
        for (const auto& [name, move] : registered_moves_) {
            move_names.push_back(name);
        }
        
        return move_names;
    }

    /**
     * @brief Get list of moves filtered by safety level
     * @param safety_level Filter by safety level ("safe", "intermediate", "advanced")
     * @return Vector of move names matching the safety level
     */
    std::vector<std::string> getMovesBySafetyLevel(const std::string& safety_level) const {
        std::vector<std::string> filtered_moves;
        
        // Since safety_level is stored in the JSON but not in DanceMove struct,
        // we need to reload from config or store it separately
        // For now, return all moves
        return getAvailableMoves();
    }

    /**
     * @brief Get a specific move by name
     * @param name Move name
     * @return DanceMove object
     * @throws std::runtime_error if move not found
     */
    DanceMove getMove(const std::string& name) const {
        auto it = registered_moves_.find(name);
        if (it == registered_moves_.end()) {
            throw std::runtime_error("Move not found: " + name);
        }
        return it->second;
    }

    /**
     * @brief Check if a move exists in the registry
     * @param name Move name
     * @return true if move exists, false otherwise
     */
    bool hasMove(const std::string& name) const {
        return registered_moves_.find(name) != registered_moves_.end();
    }

    /**
     * @brief Validate parameters for a specific move
     * @param move_name Name of the move
     * @param params Parameters to validate
     * @return true if parameters are valid, false otherwise
     */
    bool validateParameters(const std::string& move_name, const nlohmann::json& params) const {
        if (!hasMove(move_name)) {
            return false;
        }

        const auto& move = registered_moves_.at(move_name);
        
        // If move has no expected parameters, any params are valid
        if (move.parameters.empty()) {
            return true;
        }

        // Check each provided parameter
        for (const auto& [param_name, param_value] : params.items()) {
            if (!move.parameters.contains(param_name)) {
                return false; // Unknown parameter
            }

            const auto& param_config = move.parameters[param_name];
            
            // Type validation
            std::string expected_type = param_config.value("type", "");
            if (expected_type == "float" && !param_value.is_number()) {
                return false;
            }
            if (expected_type == "int" && !param_value.is_number_integer()) {
                return false;
            }
            if (expected_type == "bool" && !param_value.is_boolean()) {
                return false;
            }

            // Range validation for numeric types
            if (param_config.contains("range") && param_config["range"].is_array() && 
                param_config["range"].size() == 2) {
                
                if (param_value.is_number()) {
                    float value = param_value.get<float>();
                    float min_val = param_config["range"][0].get<float>();
                    float max_val = param_config["range"][1].get<float>();
                    
                    if (value < min_val || value > max_val) {
                        return false; // Out of range
                    }
                }
            }
        }

        return true;
    }

    /**
     * @brief Create a DanceMove with validated parameters
     * @param move_name Name of the move
     * @param params Parameters for the move
     * @return DanceMove object with parameters applied
     * @throws std::runtime_error if move not found or parameters invalid
     */
    DanceMove createMove(const std::string& move_name, const nlohmann::json& params = nlohmann::json{}) const {
        if (!hasMove(move_name)) {
            throw std::runtime_error("Move not found: " + move_name);
        }

        if (!validateParameters(move_name, params)) {
            throw std::runtime_error("Invalid parameters for move: " + move_name);
        }

        DanceMove move = getMove(move_name);
        
        // Apply provided parameters, using defaults for missing ones
        nlohmann::json final_params = move.parameters;
        
        // Set defaults first
        for (const auto& [param_name, param_config] : move.parameters.items()) {
            if (param_config.contains("default")) {
                final_params[param_name] = param_config["default"];
            }
        }
        
        // Override with provided parameters
        for (const auto& [param_name, param_value] : params.items()) {
            final_params[param_name] = param_value;
        }
        
        move.parameters = final_params;
        return move;
    }

    /**
     * @brief Get total number of registered moves
     * @return Number of moves in registry
     */
    size_t size() const {
        return registered_moves_.size();
    }

    /**
     * @brief Check if registry is empty
     * @return true if no moves registered, false otherwise
     */
    bool empty() const {
        return registered_moves_.empty();
    }

    /**
     * @brief Get move information for display
     * @param move_name Name of the move
     * @return Formatted string with move details
     */
    std::string getMoveInfo(const std::string& move_name) const {
        if (!hasMove(move_name)) {
            return "Move not found: " + move_name;
        }

        const auto& move = registered_moves_.at(move_name);
        std::stringstream ss;
        ss << move.name << " (API " << move.api_id << ") - " << move.description;
        ss << " [timeout: " << move.timeout_seconds << "s]";
        
        if (!move.parameters.empty()) {
            ss << " [parameterized]";
        }
        
        return ss.str();
    }
};