/**
   \note Alghough multiple instances of WorldROSItem can be created,
   only one /clock topic can exist in a system. The current implementation
   does not consider this limitation and the clock value may be inconsistent
   when multiple WorldROSItem publish it simultaneously. The implementation
   should be improved to avoid this problem.
*/

#include "WorldROSItem.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class WorldROSItem::Impl
{
public:
    WorldROSItem* self;
    WorldItem* worldItem;
    ScopedConnection simulationBarConnection;
    SimulatorItem* currentSimulatorItem;
    ScopedConnectionSet currentSimulatorItemConnections;
    
    unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher clockPublisher;
    double maxClockPublishingRate;
    double clockPublishingInterval;
    double nextClockPublishingTime;
    
    Impl(WorldROSItem* self);
    Impl(WorldROSItem* self, const Impl& org);
    ~Impl();
    void initialize();
    void initializeWorld(WorldItem* worldItem);
    void clearWorld();
    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void setCurrentSimulatorItem(SimulatorItem* simulatorItem);
    void clearCurrentSimulatorItem();
    void onSimulationStarted();
    void onSimulationStep();
    void onSimulationFinished();
};

}

void WorldROSItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<WorldROSItem>(N_("WorldROSItem"));
    ext->itemManager().addCreationPanel<WorldROSItem>();
}


WorldROSItem::WorldROSItem()
{
    impl = new Impl(this);
}


WorldROSItem::Impl::Impl(WorldROSItem* self)
    : self(self)
{
    maxClockPublishingRate = 100.0;
    initialize();
}


WorldROSItem::WorldROSItem(const WorldROSItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


WorldROSItem::Impl::Impl(WorldROSItem* self, const Impl& org)
    : self(self)
{
    maxClockPublishingRate = org.maxClockPublishingRate;
    initialize();
}


void WorldROSItem::Impl::initialize()
{
    simulationBarConnection =
        SimulationBar::instance()->sigSimulationAboutToStart().connect(
            [&](SimulatorItem* simulatorItem){
                onSimulationAboutToStart(simulatorItem); });
}


WorldROSItem::~WorldROSItem()
{
    delete impl;
}


WorldROSItem::Impl::~Impl()
{
    clearCurrentSimulatorItem();
    clearWorld();
}


Item* WorldROSItem::doDuplicate() const
{
    return new WorldROSItem(*this);
}


void WorldROSItem::onPositionChanged()
{
    auto worldItem = findOwnerItem<WorldItem>();
    if(worldItem){
        if(worldItem != impl->worldItem){
            impl->initializeWorld(worldItem);
        }
    } else {
        impl->clearWorld();
    }
}


void WorldROSItem::onDisconnectedFromRoot()
{
    impl->clearWorld();
}


void WorldROSItem::Impl::initializeWorld(WorldItem* worldItem)
{
    clearWorld();

    this->worldItem = worldItem;

    string name = worldItem->name();
    std::replace(name.begin(), name.end(), '-', '_');
    rosNode.reset(new ros::NodeHandle(name));

    clockPublisher = rosNode->advertise<rosgraph_msgs::Clock>("/clock", 1);
}


void WorldROSItem::Impl::clearWorld()
{
    rosNode.reset();
}


void WorldROSItem::Impl::onSimulationAboutToStart(SimulatorItem* simulatorItem)
{
    if(worldItem && worldItem == simulatorItem->findOwnerItem<WorldItem>()){
        if(simulatorItem != currentSimulatorItem){
            setCurrentSimulatorItem(simulatorItem);
            return;
        }
    }
    clearCurrentSimulatorItem();
}


void WorldROSItem::Impl::setCurrentSimulatorItem(SimulatorItem* simulatorItem)
{
    clearCurrentSimulatorItem();
    currentSimulatorItem = simulatorItem;

    currentSimulatorItemConnections.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));

    currentSimulatorItemConnections.add(
        simulatorItem->sigSimulationFinished().connect(
            [&](){ clearCurrentSimulatorItem(); }));
}


void WorldROSItem::Impl::clearCurrentSimulatorItem()
{
    currentSimulatorItemConnections.disconnect();
    currentSimulatorItem = nullptr;
}


void WorldROSItem::Impl::onSimulationStarted()
{
    clockPublishingInterval = 1.0 / maxClockPublishingRate;
    nextClockPublishingTime = 0.0;

    currentSimulatorItem->addMidDynamicsFunction(
        [&](){ onSimulationStep(); });
}


void WorldROSItem::Impl::onSimulationStep()
{
    double time = currentSimulatorItem->simulationTime();
    
    // Publish clock
    if(time >= nextClockPublishingTime){
        rosgraph_msgs::Clock clock;
        clock.clock.fromSec(time);
        clockPublisher.publish(clock);
        nextClockPublishingTime += clockPublishingInterval;
    }
}


void WorldROSItem::doPutProperties(PutPropertyFunction& putProperty)
{
  putProperty.decimals(2).min(0.0)
      (_("Max clock publishing rate"), impl->maxClockPublishingRate,
       [&](double v){ impl->maxClockPublishingRate = v; return true; });
}


bool WorldROSItem::store(Archive& archive)
{
    archive.write("maxClockPublishingRate", impl->maxClockPublishingRate);
    return true;
}


bool WorldROSItem::restore(const Archive& archive)
{
    archive.read("maxClockPublishingRate", impl->maxClockPublishingRate);
    return true;
}
