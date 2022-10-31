#include "../plugin/ROS2Plugin.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cnoid/App>
#include <cnoid/Config>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/Plugin>
#include <cnoid/PluginManager>
#include <cnoid/ProjectManager>
#include <cnoid/UTF8>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <regex>

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

int main(int argc, char** argv)
{
    regex remapPattern("^.+:=.+$");
    vector<char*> args{argv[0]};
    vector<char*> rosargs{argv[0]};
    for (int i = 1; i < argc; ++i) {
        if (regex_match(argv[i], remapPattern)) {
            rosargs.push_back(argv[i]);
        } else {
            args.push_back(argv[i]);
        }
    }

    int rosargc = rosargs.size();
    char** rosargv = &rosargs.front();
    rclcpp::init(rosargc, rosargv);

    argc = args.size();
    argv = &args.front();
    cnoid::App app(argc, argv, "Choreonoid-ROS2", "Choreonoid");

    auto plugin_manager = cnoid::PluginManager::instance();

    if(auto pluginPath = getenv("CNOID_PLUGIN_PATH")){
      plugin_manager->addPluginPath(toUTF8(pluginPath));
    }

    plugin_manager->addPluginPath(
        ament_index_cpp::get_package_share_directory("choreonoid_ros2")
        + "/../../lib/choreonoid-1.8");

    app.requirePluginToCustomizeApplication("ROS2");

    ROS2Plugin* ros2Plugin = static_cast<ROS2Plugin*>(plugin_manager->findPlugin("ROS2"));

    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node;
    node.reset(ros2Plugin);
    executor.add_node(node);

    int ret = app.exec();

    rclcpp::shutdown();

    return ret;
}
