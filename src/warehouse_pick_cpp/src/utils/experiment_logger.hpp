#ifndef EXPERIMENT_LOGGER_HPP
#define EXPERIMENT_LOGGER_HPP

#include <string>
#include <fstream>

class ExperimentLogger {
public:
    ExperimentLogger(const std::string& filepath);
    ~ExperimentLogger();
    
    // Log a single planning attempt
    void log_attempt(
        int trial,
        int attempt,
        bool success,
        double planning_time,
        double path_length,
        double smoothness,
        double total_cost
    );
    
    // Log best trajectory from a trial
    void log_best(
        int trial,
        bool found_solution,
        double best_cost,
        double best_length,
        int best_attempt
    );
    
    void flush();
    
private:
    std::ofstream file_;
};

#endif