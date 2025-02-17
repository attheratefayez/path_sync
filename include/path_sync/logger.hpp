/**
 * @file logger.hpp
 * @brief Header file for Logger Class for PathSync Project.  
 *
 * LOG_LEVELS: 
 *  - info()
 *  - warn()
 *  - error()
 *
 */

#ifndef __PATH_SYNC_LOGGER_HPP__
#define __PATH_SYNC_LOGGER_HPP__

#define RESET "\033[0m"
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */

namespace psync{
/**
 * @class Logger
 * @brief Singleton class for logging functionalities in pathsync project.
 *
 * LOG_LEVELS : 
 *    - info
 *    - warn
 *    - error
 */
class Logger {
public:

    /**
     * @brief Get the singleton instance of Logger.
     *
     * @return Logger*
     */
    static Logger* get();

    Logger(Logger const&) = delete;
    void operator=(Logger const&) = delete;

    /**
     * @brief log an info message.
     *
     * @param msg Message to log.
     */
    void info(const char* msg);


    /**
     * @brief log an warn message.
     *
     * @param msg Message to log.
     */
    void warn(const char* msg);


    /**
     * @brief log an error message.
     *
     * @param msg Message to log.
     */
    void error(const char* msg);

private:
    static Logger* __instance;
    Logger();

    ~Logger();
};

}

#endif // !__PATH_SYNC_LOGGER_HPP__
