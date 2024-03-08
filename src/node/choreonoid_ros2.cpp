#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cnoid/App>
#include <cnoid/ExecutablePath>
#include <cnoid/Plugin>
#include <cnoid/PluginManager>
#include <cnoid/ProjectManager>
#include <cnoid/UTF8>
#include <cstdlib>

int main(int argc, char** argv)
{
    cnoid::App app(argc, argv, "Choreonoid-ROS2", "Choreonoid");

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
