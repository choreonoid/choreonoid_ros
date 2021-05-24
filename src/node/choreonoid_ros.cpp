#include <cnoid/Config>
#include <cnoid/App>
#include <cnoid/ProjectManager>
#include <cnoid/PluginManager>
#include <cnoid/Plugin>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/spinner.h>
#include <regex>
#include <cstdlib>

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
    ros::init(rosargc, rosargv, "choreonoid", ros::init_options::NoSigintHandler);

    ros::master::setRetryTimeout(ros::WallDuration(1));
    if(!ros::master::check()){
        cerr << "Choreonoid's ROS node cannot be invoked because the ROS master is not found." << endl;
        return 1;
    }
    std::unique_ptr<ros::AsyncSpinner> spinner;
    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    argc = args.size();
    argv = &args.front();
    cnoid::App app(argc, argv);
    app.initialize("Choreonoid-ROS", "Choreonoid");
    ProjectManager::instance()->loadBuiltinProject(":/Base/project/layout.cnoid");

    auto pm = PluginManager::instance();
    auto rosPlugin = pm->findPlugin("ROS");
    if(!rosPlugin){
        auto& errorMessage = pm->getErrorMessage("ROS");
        if(errorMessage.empty()){
            cerr << "ROS plugin is not found." << endl;
        } else {
            cerr << "ROS plugin cannot be loaded.\n";
            cerr << errorMessage << endl;
        }
        return 1;
    }
    if(!rosPlugin->isActive()){
        cerr << "ROS plugin is not active." << endl;
        return 1;
    }
    
    app.exec();

    ros::requestShutdown();
    ros::waitForShutdown();

    return 0;
}
