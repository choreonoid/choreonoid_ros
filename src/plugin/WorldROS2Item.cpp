/**
   \note Alghough multiple instances of WorldROSItem can be created,
   only one /clock topic can exist in a system. The current implementation
   does not consider this limitation and the clock value may be inconsistent
   when multiple WorldROSItem publish it simultaneously. The implementation
   should be improved to avoid this problem.
*/

#include "WorldROS2Item.h"
#include "gettext.h"
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std;
using namespace cnoid;

namespace cnoid {

class WorldROS2Item::Impl : public rclcpp::Node {
public:
  WorldROS2Item *self;
  WorldItem *worldItem;
  ScopedConnection simulationBarConnection;
  SimulatorItem *currentSimulatorItem;
  ScopedConnectionSet currentSimulatorItemConnections;

  //    unique_ptr<ros::NodeHandle> rosNode;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher;
  double maxClockPublishingRate;
  double clockPublishingInterval;
  double nextClockPublishingTime;

  Impl(WorldROS2Item *self);
  Impl(WorldROS2Item *self, const Impl &org);
  ~Impl();
  void initialize();
  void initializeWorld(WorldItem *worldItem);
  void initializeClockPublisher();
  void clearWorld();
  void onSimulationAboutToStart(SimulatorItem *simulatorItem);
  void setCurrentSimulatorItem(SimulatorItem *simulatorItem);
  void clearCurrentSimulatorItem();
  void onSimulationStarted();
  void onSimulationStep();
  void onSimulationFinished();
};

} // namespace cnoid

void WorldROS2Item::initializeClass(ExtensionManager *ext) {
  ext->itemManager().registerClass<WorldROS2Item>(N_("WorldROS2Item"));
  ext->itemManager().addCreationPanel<WorldROS2Item>();
}

WorldROS2Item::WorldROS2Item() { impl = new Impl(this); }

WorldROS2Item::Impl::Impl(WorldROS2Item *self)
    : rclcpp::Node("world_ros", rclcpp::NodeOptions()), self(self) {
  maxClockPublishingRate = 100.0;
  initialize();
}

WorldROS2Item::WorldROS2Item(const WorldROS2Item &org) : Item(org) {
  impl = new Impl(this, *org.impl);
}

WorldROS2Item::Impl::Impl(WorldROS2Item *self, const Impl &org)
    : rclcpp::Node("world_ros", rclcpp::NodeOptions()), self(self) {
  maxClockPublishingRate = org.maxClockPublishingRate;
  initialize();
}

void WorldROS2Item::Impl::initialize() {
  simulationBarConnection =
      SimulationBar::instance()->sigSimulationAboutToStart().connect(
          [&](SimulatorItem *simulatorItem) {
            onSimulationAboutToStart(simulatorItem);
          });
}

WorldROS2Item::~WorldROS2Item() { delete impl; }

WorldROS2Item::Impl::~Impl() {
  clearCurrentSimulatorItem();
  clearWorld();
}

Item *WorldROS2Item::doDuplicate() const { return new WorldROS2Item(*this); }

void WorldROS2Item::onPositionChanged() {
  WorldItem *worldItem = nullptr;
  if (isConnectedToRoot()) {
    worldItem = findOwnerItem<WorldItem>();
    if (worldItem && worldItem != impl->worldItem) {
      impl->initializeWorld(worldItem);
    }
  }
  if (!worldItem) {
    impl->clearWorld();
  }
}

void WorldROS2Item::Impl::initializeWorld(WorldItem *worldItem) {
  clearWorld();

  this->worldItem = worldItem;

  string name = worldItem->name();
  std::replace(name.begin(), name.end(), '-', '_');
  //    rosNode.reset(new ros::NodeHandle(name));

  initializeClockPublisher();
}

void WorldROS2Item::Impl::initializeClockPublisher() {
  clockPublisher.reset();
  if (maxClockPublishingRate > 0.0) {
    clockPublisher = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  }
}

void WorldROS2Item::setMaxClockPublishingRate(double rate) {
  impl->maxClockPublishingRate = rate;
  impl->initializeClockPublisher();
}

void WorldROS2Item::Impl::clearWorld() {
  //    if(rosNode){
  //        rosNode.reset();
  //        clockPublisher.shutdown();
  //        this->;
  //    }
}

void WorldROS2Item::Impl::onSimulationAboutToStart(
    SimulatorItem *simulatorItem) {
  if (worldItem && worldItem == simulatorItem->findOwnerItem<WorldItem>()) {
    if (simulatorItem != currentSimulatorItem) {
      setCurrentSimulatorItem(simulatorItem);
      return;
    }
  }
  clearCurrentSimulatorItem();
}

void WorldROS2Item::Impl::setCurrentSimulatorItem(
    SimulatorItem *simulatorItem) {
  clearCurrentSimulatorItem();
  currentSimulatorItem = simulatorItem;

  currentSimulatorItemConnections.add(
      simulatorItem->sigSimulationStarted().connect(
          [&]() { onSimulationStarted(); }));

  currentSimulatorItemConnections.add(
      simulatorItem->sigSimulationFinished().connect(
          [&](bool /* isForced */) { clearCurrentSimulatorItem(); }));
}

void WorldROS2Item::Impl::clearCurrentSimulatorItem() {
  currentSimulatorItemConnections.disconnect();
  currentSimulatorItem = nullptr;
}

void WorldROS2Item::Impl::onSimulationStarted() {
  clockPublishingInterval = 1.0 / maxClockPublishingRate;
  nextClockPublishingTime = 0.0;

  currentSimulatorItem->addMidDynamicsFunction([&]() { onSimulationStep(); });
}

void WorldROS2Item::Impl::onSimulationStep() {
  double time = currentSimulatorItem->simulationTime();

  // Publish clock
  if (time >= nextClockPublishingTime) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock.set__sec(static_cast<int>(time));
    int nanosec = (time - clock.clock.sec) * 10e9;
    clock.clock.set__nanosec(nanosec);
    clockPublisher->publish(clock);
    nextClockPublishingTime += clockPublishingInterval;
  }
}

void WorldROS2Item::doPutProperties(PutPropertyFunction &putProperty) {
  putProperty.decimals(2).min(0.0)(_("Max clock publishing rate"),
                                   impl->maxClockPublishingRate, [&](double r) {
                                     setMaxClockPublishingRate(r);
                                     return true;
                                   });
}

bool WorldROS2Item::store(Archive &archive) {
  archive.write("max_clock_publishing_rate", impl->maxClockPublishingRate);
  return true;
}

bool WorldROS2Item::restore(const Archive &archive) {
  archive.read("max_clock_publishing_rate", impl->maxClockPublishingRate);
  return true;
}
