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

void Logger::log_in_file_(const std::stringstream& ss)
{
    if(log_file_.empty())
        log_file_ = std::string( PROJECT_ROOT ) + "/log/log_file.txt";
    std::ofstream log_stream(log_file_, std::ios_base::ate);

    while(log_stream.is_open())
    {
        log_stream << ss.str() ;
        log_stream.close();
    }
}

Logger& Logger::get()
{
    static Logger instance;
    return instance;
}

void Logger::info(const char *msg)
{
    std::stringstream ss;
    ss << GREEN << BOLD << "[ INFO ]  " << RESET << msg << "\n";
    std::cout << ss.str();

    if(not log_file_.empty())
        log_in_file_(ss);
}

void Logger::warn(const char *msg)
{
    std::stringstream ss;
    ss << YELLOW << BOLD << "[ WARN ]  " << RESET << msg << "\n";
    std::cout << ss.str();

    if(not log_file_.empty())
        log_in_file_(ss);
}

void Logger::error(const char *msg)
{
    std::stringstream ss;
    ss << RED << BOLD << "[ ERROR ] " << RESET << msg << "\n";
    std::cerr << ss.str();

    if(not log_file_.empty())
        log_in_file_(ss);

    exit(0);
}

} // namespace psync
