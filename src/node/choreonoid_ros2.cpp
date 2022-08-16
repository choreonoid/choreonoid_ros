#include <cnoid/Config>
#include <cnoid/App>
#include <cnoid/ProjectManager>
#include <cnoid/PluginManager>
#include <cnoid/Plugin>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <rclcpp/rclcpp.hpp>
#include "../plugin/ROS2Plugin.hpp"
#include <regex>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

int main(int argc, char** argv)
{
    regex remapPattern("^.+:=.+$");
    vector<char*> args{ argv[0] };
    vector<char*> rosargs{ argv[0] };
    for(int i=1; i < argc; ++i){
        if(regex_match(argv[i], remapPattern)){
            rosargs.push_back(argv[i]);
        } else {
            args.push_back(argv[i]);
        }
    }

    int rosargc = rosargs.size();
    char** rosargv = &rosargs.front();
    rclcpp::init(rosargc, rosargv);
//    ros::init(rosargc, rosargv, "choreonoid", ros::init_options::NoSigintHandler);

//    ros::master::setRetryTimeout(ros::WallDuration(1));



//    std::unique_ptr<ros::AsyncSpinner> spinner;
//    spinner.reset(new ros::AsyncSpinner(0));
//    spinner->start();

    argc = args.size();
    argv = &args.front();
    cnoid::App app(argc, argv,"Choreonoid-ROS2", "Choreonoid");
    PluginManager::instance()->addPluginPath(ament_index_cpp::get_package_share_directory("choreonoid_ros")+"/../../lib");
    app.initialize();
    ProjectManager::instance()->loadBuiltinProject(":/Base/project/layout.cnoid");

    auto ros2PluginRaw = PluginManager::instance()->findPlugin("ROS2");
    ROS2Plugin *ros2Plugin = static_cast<ROS2Plugin*>(ros2PluginRaw);

    if(!ros2Plugin){
//        auto& errorMessage = ProjectManager::instance()->getErrorMessage("ROS2");
//        if(errorMessage.empty()){
//            cerr << "ROS2 plugin is not found." << endl;
//        } else {
//            cerr << "ROS2 plugin cannot be loaded.\n";
//            cerr << errorMessage << endl;
//        }
        return 1;
    }

    using rclcpp::executors::MultiThreadedExecutor;
    MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node;
    node.reset(ros2Plugin);
    executor.add_node(node);

    if(!ros2Plugin->isActive()){
        cerr << "ROS2 plugin is not active." << endl;
        return 1;
    }
    
    app.exec();

    rclcpp::shutdown();

    return 0;
}
