#include <cnoid/Config>
#include <cnoid/App>
#include <cstdlib>
//#include <ros/ros.h>

using namespace std;
using namespace cnoid;

int main(int argc, char **argv)
{
    cnoid::App app(argc, argv);

    app.initialize("Choreonoid", "Choreonoid", getenv("CNOID_PLUGIN_PATH"));
    app.exec();

    return 0;
}
