#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <string>
#include <vector>

struct ExperimentConfig {
    // Planner settings
    bool multi_start_enabled;
    int num_attempts;
    double planning_time_sec;
    
    // Goal configuration
    double goal_x;
    double goal_y;
    double goal_z;
    
    // Start configuration
    std::vector<double> start_joints;
    
    // Logging
    std::string output_dir;
    std::string csv_filename;
    
    // Load from YAML file
    static ExperimentConfig load(const std::string& yaml_path);
    
    // Print config
    void print() const;
};

#endif
