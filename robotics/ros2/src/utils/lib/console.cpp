/*! @package utils
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "utils/console.hpp"

bool fileExist(std::string path) { return !(access(path.c_str(), R_OK) < 0); }

std::string executeShellCommand(std::string command)
{
    std::array<char, 1024> buffer;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    std::string out = "";
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }

    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 1024, pipe.get()) != NULL)
        {
            out = out + buffer.data();
        }
    }

    if (out.compare("") != 0)
    {
        out = out.substr(0, out.size() - 1);
    }
    return out;
}

extern const char *bool2c(bool var) { return var ? "TRUE" : "FALSE"; }

extern int getEnv(const char *var, int default_var)
{
    try
    {
        return std::stoi(getenv(var));
    }
    catch (const std::exception &e)
    {
        return default_var;
    }
}

extern std::string getEnv(const char *var, const char *default_var)
{
    try
    {
        return std::string(getenv(var));
    }
    catch (const std::exception &e)
    {
        return default_var;
    }
}

extern bool getEnv(const char *var, bool default_var)
{
    try
    {
        return std::string(getenv(var)).compare("1") == 0;
    }
    catch (const std::exception &e)
    {
        return default_var;
    }
}

extern float getEnv(const char *var, float default_var)
{
    try
    {
        return std::stof(getenv(var));
    }
    catch (const std::exception &e)
    {
        return default_var;
    }
}