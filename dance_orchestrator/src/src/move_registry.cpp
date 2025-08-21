#include "move_registry.hpp"
#include <iostream>
#include <algorithm>

// Implementation is mostly header-only due to template nature
// This file can contain utility functions or extended functionality

namespace {
    /**
     * @brief Helper function to get safety level from config
     */
    std::string getSafetyLevelFromConfig(const nlohmann::json& config, const std::string& move_name) {
        try {
            std::ifstream file(config.dump()); // This is a placeholder
            // In practice, we'd need to re-read the config file to get safety_level
            // since it's not stored in the DanceMove struct
            return "safe"; // Default fallback
        } catch (...) {
            return "safe";
        }
    }
}

// Additional utility functions can be added here as needed