#include "utils/experiment_logger.hpp"
#include <iostream>
#include <iomanip>

ExperimentLogger::ExperimentLogger(const std::string& filepath) {
    file_.open(filepath);
    if (!file_.is_open()) {
        std::cerr << "Failed to open log file: " << filepath << std::endl;
        throw std::runtime_error("Cannot open log file");
    }
    
    // Write CSV header
    file_ << "trial,attempt,success,planning_time_sec,path_length,smoothness,total_cost\n";
}

ExperimentLogger::~ExperimentLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void ExperimentLogger::log_attempt(
    int trial,
    int attempt,
    bool success,
    double planning_time,
    double path_length,
    double smoothness,
    double total_cost)
{
    file_ << trial << ","
          << attempt << ","
          << (success ? 1 : 0) << ","
          << std::fixed << std::setprecision(4) << planning_time << ","
          << std::fixed << std::setprecision(3) << path_length << ","
          << std::fixed << std::setprecision(3) << smoothness << ","
          << std::fixed << std::setprecision(3) << total_cost << "\n";
}

void ExperimentLogger::log_best(
    int trial,
    bool found_solution,
    double best_cost,
    double best_length,
    int best_attempt)
{
    // Could write to a separate "summary" CSV if needed
    std::cout << "Trial " << trial << ": "
              << (found_solution ? "SUCCESS" : "FAILED")
              << " (best_cost=" << best_cost 
              << ", len=" << best_length
              << ", attempt=" << best_attempt << ")" << std::endl;
}

void ExperimentLogger::flush() {
    file_.flush();
}