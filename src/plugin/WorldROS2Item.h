#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H

#include "exportdecl.h"
#include <cnoid/Item>

namespace cnoid {

class CNOID_EXPORT WorldROS2Item : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldROS2Item();
    WorldROS2Item(const WorldROS2Item& org);
    virtual ~WorldROS2Item();
    void setMaxClockPublishingRate(double rate);

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<WorldROS2Item> WorldROSItemPtr;

}  // namespace cnoid

#endif
