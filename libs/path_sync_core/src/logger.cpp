#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "path_sync_core/logger.hpp"

namespace path_sync
{

Logger::Logger()
{
}

Logger& Logger::get()
{
    static Logger instance;
    return instance;
}

void Logger::info(const char *msg, std::shared_ptr<std::ofstream> log_file)
{
    std::stringstream ss;
    ss << GREEN << BOLD << "[ INFO ]  " << RESET << msg << "\n";
    std::cout << ss.str();

    if(log_file)
        *log_file << ss.str();
}

void Logger::warn(const char *msg, std::shared_ptr<std::ofstream> log_file)
{
    std::stringstream ss;
    ss << YELLOW << BOLD << "[ WARN ]  " << RESET << msg << "\n";
    std::cout << ss.str();

    if(log_file)
        *log_file << ss.str();
}

void Logger::error(const char *msg, std::shared_ptr<std::ofstream> log_file)
{
    std::stringstream ss;
    ss << RED << BOLD << "[ ERROR ] " << RESET << msg << "\n";
    std::cerr << ss.str();

    if(log_file)
        *log_file << ss.str();

    exit(0);
}

} // namespace psync
