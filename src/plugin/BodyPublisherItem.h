#ifndef CNOID_ROS_PLUGIN_BODY_PUBLISHER_ITEM_H
#define CNOID_ROS_PLUGIN_BODY_PUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class BodyPublisherItemImpl;

class CNOID_EXPORT BodyPublisherItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyPublisherItem();
    BodyPublisherItem(const BodyPublisherItem& org);
    virtual ~BodyPublisherItem();

    virtual double timeStep() const override;
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;
    
protected:
    virtual Item* doDuplicate() const override;

    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    BodyPublisherItemImpl* impl;
};

typedef ref_ptr<BodyPublisherItem> BodyPublisherItemPtr;

}

#endif
