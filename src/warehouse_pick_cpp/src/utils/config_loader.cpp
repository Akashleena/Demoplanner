#include "utils/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

ExperimentConfig ExperimentConfig::load(const std::string& yaml_path) {
    ExperimentConfig config;
    
    try {
        YAML::Node yaml = YAML::LoadFile(yaml_path);
        
        // Planner settings
        config.multi_start_enabled = yaml["planner"]["multi_start"]["enabled"].as<bool>();
        config.num_attempts = yaml["planner"]["multi_start"]["num_attempts"].as<int>();
        config.planning_time_sec = yaml["planner"]["planning_time_sec"].as<double>();
        
        // Goal
        config.goal_x = yaml["goal"]["position"]["x"].as<double>();
        config.goal_y = yaml["goal"]["position"]["y"].as<double>();
        config.goal_z = yaml["goal"]["position"]["z"].as<double>();
        
        // Start joints
        config.start_joints = yaml["goal"]["start_joints"].as<std::vector<double>>();
        
        // Logging
        config.output_dir = yaml["logging"]["output_dir"].as<std::string>();
        config.csv_filename = yaml["logging"]["csv_filename"].as<std::string>();
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        throw;
    }
    
    return config;
}

void ExperimentConfig::print() const {
    std::cout << "=== Experiment Configuration ===" << std::endl;
    std::cout << "Multi-start: " << (multi_start_enabled ? "enabled" : "disabled") << std::endl;
    std::cout << "Attempts: " << num_attempts << std::endl;
    std::cout << "Planning time: " << planning_time_sec << "s" << std::endl;
    std::cout << "Goal: (" << goal_x << ", " << goal_y << ", " << goal_z << ")" << std::endl;
    std::cout << "Output: " << output_dir << "/" << csv_filename << std::endl;
    std::cout << "===============================" << std::endl;
}