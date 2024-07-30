#ifndef CNOID_ROS_PLUGIN_FORMAT_H
#define CNOID_ROS_PLUGIN_FORMAT_H

#include <cnoid/Config>

#if CNOID_VERSION >= 0x020200
#include <cnoid/Format>

#else

#include <fmt/format.h>

namespace cnoid {

#if __cplusplus >= 202002L && FMT_VERSION >= 80000

template<typename... Args>
std::string formatC(fmt::format_string<Args...> s, Args&&... args)
{
    return fmt::vformat(s, fmt::make_format_args(args...));
}

template<typename... Args>
std::string formatR(fmt::string_view s, Args&&... args)
{
    return fmt::format(fmt::runtime(s), args...);
}

#elif __cplusplus >= 201703L

template<typename... Args>
std::string formatC(fmt::string_view s, Args&&... args)
{
    return fmt::format(s, args...);
}

template<typename... Args>
std::string formatR(fmt::string_view s, Args&&... args)
{
    return fmt::format(s, args...);
}

#else

template<typename... Args>
std::string formatC(const char* s, Args&&... args)
{
    return fmt::format(s, args...);
}

template<typename... Args>
std::string formatC(const std::string& s, Args&&... args)
{
    return fmt::format(s, args...);
}

template<typename... Args>
std::string formatR(const char* s, Args&&... args)
{
    return fmt::format(s, args...);
}

template<typename... Args>
std::string formatR(const std::string& s, Args&&... args)
{
    return fmt::format(s, args...);
}

#endif

} // name space cnoid

#endif // CNOID_VERSION < 0x020200

#endif // CNOID_UTIL_FORMAT_H
