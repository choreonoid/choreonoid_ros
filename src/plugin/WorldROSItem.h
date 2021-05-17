#ifndef CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_WORLD_ROS_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT WorldROSItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldROSItem();
    WorldROSItem(const WorldROSItem& org);
    virtual ~WorldROSItem();
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

typedef ref_ptr<WorldROSItem> WorldROSItemPtr;

}

#endif

