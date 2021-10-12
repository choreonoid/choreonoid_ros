#include "BodyROS2Item.h"
#include "WorldROS2Item.h"
#include "deprecated/BodyPublisherItem.h"
#include <cnoid/MessageView>
#include <cnoid/Plugin>
#include <ros/init.h>

using namespace std;
using namespace cnoid;

class ROS2Plugin : public Plugin
{
public:
    ROS2Plugin() : Plugin("ROS2")
    {
        require("Body");
    }
  
    virtual bool initialize()
    {
        if(!ros::isInitialized()){
            MessageView::instance()->putln(
                "The ROS2 plugin cannot be used because ROS is not initialized. \n"
                "Choreonoid must be invoked as a ROS node to make the ROS plugin available.",
                MessageView::ERROR);
            return false;
        }

        WorldROS2Item::initializeClass(this);
        BodyROS2Item::initializeClass(this);
        BodyPublisherItem::initializeClass(this);
        
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROS2Plugin)
