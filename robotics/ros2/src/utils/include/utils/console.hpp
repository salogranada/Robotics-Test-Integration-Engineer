/*! @package utils
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef UTILS_CONSOLE_H
#define UTILS_CONSOLE_H
#include <unistd.h>
#include <algorithm>
#include <array>
#include <exception>
#include <fstream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>

/* Functions */
extern const char *bool2c(bool var);
extern bool fileExist(std::string path);
extern std::string executeShellCommand(std::string command);

/* Env. Variables Functions */
extern int getEnv(const char *var, int default_var);
extern bool getEnv(const char *var, bool default_var);
extern float getEnv(const char *var, float default_var);
extern std::string getEnv(const char *var, const char *default_var);
#endif /* End of UTILS_CONSOLE_H */