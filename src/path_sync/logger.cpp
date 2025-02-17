#include "path_sync/logger.hpp"
#include <cstdlib>
#include <iostream>

namespace psync {

Logger* Logger::__instance= nullptr;

Logger::Logger() { }

Logger* Logger::get()
{
    if (__instance == nullptr)
        __instance = new Logger;

    return __instance;
}

void Logger::info(const char* msg)
{
    std::cout << "[ INFO ]  " << msg << "\n";
}

void Logger::warn(const char* msg)
{
    std::cout << YELLOW << "[ WARN ]  " << RESET << msg << "\n";
}

void Logger::error(const char* msg)
{
    std::cerr << RED << "[ ERROR ] " << RESET << msg << "\n";
    exit(0);
}

Logger::~Logger()
{
    delete __instance;
}

} // End of namespace momapf
