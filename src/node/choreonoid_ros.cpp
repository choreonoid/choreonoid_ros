#include <cnoid/Config>
#include <cnoid/App>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include <cstdlib>

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

int main(int argc, char **argv)
{
    cnoid::App app(argc, argv);

    auto rosPluginPath = (filesystem::path(executablePath()).parent_path() / CNOID_VERSION_SUBDIR).string();
    auto pluginPathVariable = getenv("CNOID_PLUGIN_PATH");
    string pluginPathList;
    if(!pluginPathVariable){
        pluginPathList = rosPluginPath;
    } else {
        pluginPathList = format("{0}{1}{2}", pluginPathVariable, PATH_DELIMITER, rosPluginPath);
    }

    app.initialize("Choreonoid", "Choreonoid", pluginPathList.c_str());
    app.exec();

    return 0;
}
