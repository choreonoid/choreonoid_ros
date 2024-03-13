#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/utilities.hpp>

#include <cnoid/App>
#include <cnoid/ExecutablePath>
#include <cnoid/Plugin>
#include <cnoid/PluginManager>
#include <cnoid/ProjectManager>
#include <cnoid/UTF8>

#include <string>
#include <vector>
#include <cstdlib>

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

    auto plugin_manager = cnoid::PluginManager::instance();

    if(auto pluginPath = getenv("CNOID_PLUGIN_PATH")){
      plugin_manager->addPluginPath(cnoid::toUTF8(pluginPath));
    }

    auto plugin_path = ament_index_cpp::get_package_share_directory("choreonoid_ros") + "/../../lib/choreonoid_ros";
    plugin_manager->addPluginPath(plugin_path);

    app.requirePluginToCustomizeApplication("ROS2");

    int ret = app.exec();

    return ret;
}
