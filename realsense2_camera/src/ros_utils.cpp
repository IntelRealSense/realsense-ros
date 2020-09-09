#include <ros_utils.h>
#include <algorithm>
#include <map>
#include <cctype>

namespace realsense2_camera
{

template <typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m)
{
    os << '{';
    for (const auto& kv : m)
    {
        os << " {" << kv.first << ": " << kv.second << '}';
    }
    os << " }";
    return os;
}

}

/**
 * Same as ros::names::isValidCharInName, but re-implemented here because it's not exposed.
 */
bool isValidCharInName(char c)
{
    return std::isalnum(c) || c == '/' || c == '_';
}

/**
 * ROS Graph Resource names don't allow spaces and hyphens (see http://wiki.ros.org/Names),
 * so we replace them here with underscores.
 */

std::string create_graph_resource_name(const std::string &original_name)
{
std::string fixed_name = original_name;
std::transform(fixed_name.begin(), fixed_name.end(), fixed_name.begin(),
                [](unsigned char c) { return std::tolower(c); });
std::replace_if(fixed_name.begin(), fixed_name.end(), [](const char c) { return !isValidCharInName(c); },
                '_');
return fixed_name;
}
