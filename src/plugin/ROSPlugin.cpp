#include "BodyPublisherItem.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/spinner.h>

using namespace std;
using namespace cnoid;

class ROSPlugin : public Plugin
{
    std::unique_ptr<ros::AsyncSpinner> spinner;
    
public:
    ROSPlugin() : Plugin("ROS") {
        require("Body");
    }
  
    virtual bool initialize()
    {
        int argc = 0;
        char** argv = 0;

        if(!ros::isInitialized()){
            ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
        }

        if(!ros::master::check()){
            MessageView::instance()->putln(
                MessageView::WARNING, "The ROS master is not found.");
            return false;
        }
            
        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();

        BodyPublisherItem::initialize(this);
        
        return true;
    }

    virtual bool finalize()
    {
        ros::requestShutdown();
        ros::waitForShutdown();
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSPlugin)
