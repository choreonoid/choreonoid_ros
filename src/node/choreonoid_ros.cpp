#include "../util/ROSUtil.h"
#include <cnoid/Config>
#include <cnoid/App>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <regex>
#include <cstdlib>

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

int main(int argc, char **argv)
{
    regex remapPattern("^.+:=.+$");
    vector<char*> args{ argv[0] };
    auto& rosargs = cnoid::rosInitArguments();
    rosargs.push_back(argv[0]);
    for(int i=1; i < argc; ++i){
        if(regex_match(argv[i], remapPattern)){
            rosargs.push_back(argv[i]);
        } else {
            args.push_back(argv[i]);
        }
    }

    argc = args.size();
    argv = &args.front();
    cnoid::App app(argc, argv);

    auto rosPluginPath = (filesystem::path(executablePath()).parent_path() / CNOID_VERSION_SUBDIR).string();
    auto pluginPathVariable = getenv("CNOID_PLUGIN_PATH");
    string pluginPathList;
    if(!pluginPathVariable){
        pluginPathList = rosPluginPath;
    } else {
        pluginPathList = string(pluginPathVariable) + PATH_DELIMITER + rosPluginPath;
    }

    app.initialize("Choreonoid", "Choreonoid", pluginPathList.c_str());
    app.exec();

    return 0;
}
