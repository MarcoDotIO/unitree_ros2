#pragma once

#include <string>
#include <cstdint>
#include "nlohmann/json.hpp"

/**
 * @brief Represents a single dance move with its parameters and metadata
 */
struct DanceMove {
    std::string name;           // "dance1", "front_flip", etc.
    int32_t api_id;            // Sport API ID
    nlohmann::json parameters; // Move-specific params
    float timeout_seconds;     // Max execution time
    std::string description;   // Human-readable description
    float delay_after;         // Delay after this move in seconds

    // Default constructor
    DanceMove() = default;

    // Constructor with parameters
    DanceMove(const std::string& name, int32_t api_id, float timeout = 5.0f, 
              const std::string& description = "", const nlohmann::json& params = nlohmann::json{}, float delay = 0.0f)
        : name(name), api_id(api_id), parameters(params), timeout_seconds(timeout), description(description), delay_after(delay) {}

    // JSON serialization
    nlohmann::json toJson() const {
        return nlohmann::json{
            {"name", name},
            {"api_id", api_id},
            {"parameters", parameters},
            {"timeout", timeout_seconds},
            {"description", description},
            {"delay_after", delay_after}
        };
    }

    // JSON deserialization
    void fromJson(const nlohmann::json& json) {
        name = json.at("name").get<std::string>();
        api_id = json.at("api_id").get<int32_t>();
        timeout_seconds = json.at("timeout").get<float>();
        description = json.value("description", "");
        parameters = json.value("parameters", nlohmann::json{});
        delay_after = json.value("delay_after", 0.0f);
    }

    // Equality operator for testing
    bool operator==(const DanceMove& other) const {
        return name == other.name && 
               api_id == other.api_id && 
               parameters == other.parameters &&
               timeout_seconds == other.timeout_seconds &&
               description == other.description &&
               delay_after == other.delay_after;
    }
};
