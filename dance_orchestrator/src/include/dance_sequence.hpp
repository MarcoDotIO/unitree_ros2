#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include "dance_move.hpp"
#include "nlohmann/json.hpp"

/**
 * @brief Represents a sequence of dance moves with timing and metadata
 */
class DanceSequence {
private:
    std::string name_;
    std::vector<DanceMove> moves_;
    float inter_move_delay_;   // Pause between moves in seconds
    std::string created_timestamp_;
    std::string version_;

public:
    // Constructors
    DanceSequence() : inter_move_delay_(1.0f), version_("1.0") {}
    
    explicit DanceSequence(const std::string& name, float inter_move_delay = 1.0f) 
        : name_(name), inter_move_delay_(inter_move_delay), version_("1.0") {
        // Set creation timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
        created_timestamp_ = ss.str();
    }

    // Getters
    const std::string& getName() const { return name_; }
    const std::vector<DanceMove>& getMoves() const { return moves_; }
    float getInterMoveDelay() const { return inter_move_delay_; }
    const std::string& getVersion() const { return version_; }
    const std::string& getCreatedTimestamp() const { return created_timestamp_; }
    size_t size() const { return moves_.size(); }
    bool empty() const { return moves_.empty(); }

    // Setters
    void setName(const std::string& name) { name_ = name; }
    void setInterMoveDelay(float delay) { inter_move_delay_ = delay; }

    // Move management
    void addMove(const DanceMove& move) {
        moves_.push_back(move);
    }

    void insertMove(size_t index, const DanceMove& move) {
        if (index > moves_.size()) {
            throw std::out_of_range("Index out of range");
        }
        moves_.insert(moves_.begin() + index, move);
    }

    void removeMove(size_t index) {
        if (index >= moves_.size()) {
            throw std::out_of_range("Index out of range");
        }
        moves_.erase(moves_.begin() + index);
    }

    void clear() {
        moves_.clear();
    }

    const DanceMove& getMove(size_t index) const {
        if (index >= moves_.size()) {
            throw std::out_of_range("Index out of range");
        }
        return moves_[index];
    }

    // File operations
    void saveToFile(const std::string& filepath) const {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for writing: " + filepath);
        }
        
        nlohmann::json json_data = toJson();
        file << json_data.dump(2);  // Pretty print with 2-space indentation
        file.close();
    }

    static DanceSequence loadFromFile(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file for reading: " + filepath);
        }

        nlohmann::json json_data;
        file >> json_data;
        file.close();

        DanceSequence sequence;
        sequence.fromJson(json_data);
        return sequence;
    }

    // JSON serialization
    nlohmann::json toJson() const {
        nlohmann::json moves_array = nlohmann::json::array();
        for (const auto& move : moves_) {
            moves_array.push_back(move.toJson());
        }

        return nlohmann::json{
            {"name", name_},
            {"version", version_},
            {"created", created_timestamp_},
            {"inter_move_delay", inter_move_delay_},
            {"moves", moves_array}
        };
    }

    // JSON deserialization
    void fromJson(const nlohmann::json& json) {
        name_ = json.at("name").get<std::string>();
        version_ = json.value("version", "1.0");
        created_timestamp_ = json.value("created", "");
        inter_move_delay_ = json.value("inter_move_delay", 1.0f);
        
        moves_.clear();
        if (json.contains("moves") && json["moves"].is_array()) {
            for (const auto& move_json : json["moves"]) {
                DanceMove move;
                move.fromJson(move_json);
                moves_.push_back(move);
            }
        }
    }

    // Utility methods
    float getTotalEstimatedDuration() const {
        float total = 0.0f;
        for (const auto& move : moves_) {
            total += move.timeout_seconds;
        }
        if (moves_.size() > 1) {
            total += (moves_.size() - 1) * inter_move_delay_;
        }
        return total;
    }

    std::string getSummary() const {
        std::stringstream ss;
        ss << "Sequence: " << name_ << " (" << moves_.size() << " moves, ~" 
           << getTotalEstimatedDuration() << "s)";
        return ss.str();
    }
};
