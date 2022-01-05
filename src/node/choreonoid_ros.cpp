#include <cnoid/App>
#include <iostream>

using namespace std;
using namespace cnoid;

int main(int argc, char** argv)
{
    cnoid::App app(argc, argv, "Choreonoid-ROS", "Choreonoid");

    if(!app.requirePluginToCustomizeApplication("ROS")){
        if(app.error() == App::PluginNotFound){
            auto message = app.errorMessage();
            if(message.empty()){
                cerr << "ROS plugin is not found." << endl;
            } else {
                cerr << "ROS plugin cannot be loaded.\n";
                cerr << message << endl;
            }
        }
        return 1;
    }
        
    return app.exec();
}
