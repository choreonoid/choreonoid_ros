#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/utilities.hpp>

#include <cnoid/App>
#include <cnoid/PluginManager>
#include <cnoid/UTF8>

#include <string>
#include <vector>
#include <iostream>

int main(int argc, char** argv)
{
    // removes ros-dependent arguments
    // rclcpp::remove_ros_arguments throws an error if it fails parsing
    std::vector<std::string> nonRosArgvString = rclcpp::remove_ros_arguments(argc, argv);
    int nonRosArgc = nonRosArgvString.size();
    char* nonRosArgv[nonRosArgc];
    for (int i = 0; i < nonRosArgc; ++i) {
        nonRosArgv[i] = nonRosArgvString.at(i).data();
    }
    if (nonRosArgvString.at(nonRosArgc - 1).empty()) {
        // ignores the final nonRosArgv
        // because remove_ros_arguments potentially returns an empty string,
        // which causes [Warning:  Input file "" was not processed.] on Choreonoid
        --nonRosArgc;
    }

    cnoid::App app(nonRosArgc, nonRosArgv, "Choreonoid-ROS2", "Choreonoid");

    auto pluginManager = cnoid::PluginManager::instance();

    const char* prefixVar = getenv("AMENT_PREFIX_PATH");
    if(prefixVar){
        do {
            const char* begin = prefixVar;
            while(*prefixVar != ':' && *prefixVar) prefixVar++;
            pluginManager->addPluginDirectoryAsPrefix(cnoid::toUTF8(std::string(begin, prefixVar)));
        } while (0 != *prefixVar++);
    } else {
        try {
            pluginManager->addPluginDirectoryAsPrefix(
                ament_index_cpp::get_package_prefix("choreonoid_ros"));
        }
        catch(const ament_index_cpp::PackageNotFoundError& ex){
            std::cerr << "Error: The choreonoid_ros package is not found." << std::endl;
            return 1;
        }
    }

    app.requirePluginToCustomizeApplication("ROS2");

    int ret = app.exec();

    return ret;
}
