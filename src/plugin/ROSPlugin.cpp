#include "WorldROSItem.h"
#include "BodyROSItem.h"
#include "ROSControlItem.h"
#include "deprecated/BodyPublisherItem.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <ros/init.h>

using namespace std;
using namespace cnoid;

class ROSPlugin : public Plugin
{
public:
    ROSPlugin() : Plugin("ROS")
    {
        require("Body");
    }
  
    virtual bool initialize()
    {
        if(!ros::isInitialized()){
            MessageView::instance()->putln(
                "The ROS plugin cannot be used because ROS is not initialized. \n"
                "Choreonoid must be invoked as a ROS node to make the ROS plugin available.",
                MessageView::ERROR);
            return false;
        }
            
        WorldROSItem::initializeClass(this);
        BodyROSItem::initializeClass(this);
        ROSControlItem::initializeClass(this);
        BodyPublisherItem::initializeClass(this);
        
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSPlugin)
