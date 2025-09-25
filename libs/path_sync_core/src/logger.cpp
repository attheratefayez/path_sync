#include <cstdlib>
#include <iostream>

#include "path_sync_core/logger.hpp"

namespace path_sync
{

Logger::Logger()
{
}

void Logger::log_in_file_(const std::stringstream& ss)
{
    std::cout << "Logging in file is not yet implemented" << std::endl;
}

Logger& Logger::get()
{
    static Logger instance;
    return instance;
}

void Logger::info(const char *msg)
{
    std::cout << GREEN << BOLD << "[ INFO ]  " << RESET << msg << "\n";
}

void Logger::warn(const char *msg)
{
    std::cout << YELLOW << BOLD << "[ WARN ]  " << RESET << msg << "\n";
}

void Logger::error(const char *msg)
{
    std::cerr << RED << BOLD << "[ ERROR ] " << RESET << msg << "\n";
    exit(0);
}

} // namespace psync
