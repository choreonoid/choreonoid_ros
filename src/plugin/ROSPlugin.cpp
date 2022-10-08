#include "WorldROSItem.h"
#include "BodyROSItem.h"
#include "ROSControlItem.h"
#include "LinkVelocityControllerROSItem.h"
#include "deprecated/BodyPublisherItem.h"
#include <cnoid/Plugin>
#include <cnoid/AppCustomizationUtil>
#include <cnoid/MessageOut>
#include <ros/init.h>
#include <ros/master.h>
#include <ros/spinner.h>
#include <memory>
#include <iostream>

using namespace std;
using namespace cnoid;

class ROSPlugin : public Plugin
{
    std::unique_ptr<ros::AsyncSpinner> spinner;
    
public:
    ROSPlugin() : Plugin("ROS")
    {
        require("Body");
    }

    bool customizeApplication(AppCustomizationUtil& app) override
    {
        ros::init(app.argc(), app.argv(), "choreonoid", ros::init_options::NoSigintHandler);
        
        // The setRetryTimeout function does not seem to affect the check function.
        // The following code is currently disabled because there seems no special reason
        // to use it other than to specify the timeout for the check function.
        //
        // ros::master::setRetryTimeout(ros::WallDuration(1));
        
        if(!ros::master::check()){
            cerr << "Choreonoid's ROS node cannot be invoked because the ROS master is not found." << endl;
            return false;
        }

        spinner.reset(new ros::AsyncSpinner(0));
        spinner->start();

        return true;
    }
  
    virtual bool initialize() override
    {
        if(!ros::isInitialized()){
            MessageOut::master()->putError(
                "The ROS plugin cannot be used because ROS is not initialized. \n"
                "Choreonoid must be invoked as a ROS node to make the ROS plugin available.\n");
            return false;
        }
            
        WorldROSItem::initializeClass(this);
        BodyROSItem::initializeClass(this);
        ROSControlItem::initializeClass(this);
        LinkVelocityControllerROSItem::initializeClass(this);
        BodyPublisherItem::initializeClass(this);
        
        return true;
    }

    virtual bool finalize() override
    {
        ros::requestShutdown();
        ros::waitForShutdown();
        spinner.reset();
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSPlugin)
