#include "BodyPublisherItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Camera>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <memory>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyNode
{
public:
    unique_ptr<ros::NodeHandle> rosNode;
    BodyItem* bodyItem;
    ScopedConnectionSet connections;
    ScopedConnection connectionOfKinematicStateChange;
    ScopedConnectionSet sensorConnections;
    TimeBar* timeBar;

    Body* ioBody;
    double time;
    double timeToPublishNext;
    double minPublishCycle;
    double timeStep;
    
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;

    DeviceList<Camera> cameras;
    vector<image_transport::Publisher> cameraImagePublishers;
    
    BodyNode(BodyItem* bodyItem);

    void start(ControllerIO* io, double maxPublishRate);
    void input();    
    void control();    
    void output();    
    void stop();    
    
    void startToPublishKinematicStateChangeOnGUI();
    void stopToPublishKinematicStateChangeOnGUI();
    void initializeJointState(Body* body);
    void publishJointState(Body* body, double time);
    void publishCameraImage(int index);
};

}

namespace cnoid {

class BodyPublisherItemImpl
{
public:
    BodyPublisherItem* self;
    unique_ptr<BodyNode> bodyNode;
    ControllerIO* io;
    double maxPublishRate;
    
    BodyPublisherItemImpl(BodyPublisherItem* self);
    BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org);
    ~BodyPublisherItemImpl();
    void setBodyItem(BodyItem* bodyItem, bool forceUpdate);
};

}


void BodyPublisherItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyPublisherItem>("BodyPublisherItem");
    ext->itemManager().addCreationPanel<BodyPublisherItem>();
}


BodyPublisherItem::BodyPublisherItem()
{
    impl = new BodyPublisherItemImpl(this);
}


BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self)
    : self(self)
{
    io = nullptr;
    maxPublishRate = 30.0;
}


BodyPublisherItem::BodyPublisherItem(const BodyPublisherItem& org)
    : ControllerItem(org)
{
    impl = new BodyPublisherItemImpl(this, *org.impl);
}
    

BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org)
    : self(self)
{
    io = nullptr;
    maxPublishRate = org.maxPublishRate;
}


BodyPublisherItem::~BodyPublisherItem()
{
    delete impl;
}


BodyPublisherItemImpl::~BodyPublisherItemImpl()
{

}


Item* BodyPublisherItem::doDuplicate() const
{
    return new BodyPublisherItem(*this);
}


void BodyPublisherItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyPublisherItemImpl::setBodyItem(BodyItem* bodyItem, bool forceUpdate)
{
    if(bodyNode){
        if(forceUpdate || bodyItem != bodyNode->bodyItem){
            bodyNode.reset();
        }
    }
    
    if(bodyItem && !bodyNode){
        bodyNode.reset(new BodyNode(bodyItem));

        bodyNode->connections.add(
            bodyItem->sigNameChanged().connect(
                [&](const std::string& oldName){ setBodyItem(bodyItem, true); }));
    }
}


void BodyPublisherItem::onDisconnectedFromRoot()
{
    impl->bodyNode.reset();
}


double BodyPublisherItem::timeStep() const
{
    return 0.0;
}


bool BodyPublisherItem::initialize(ControllerIO* io)
{
    impl->io = io;
    return true;
}


bool BodyPublisherItem::start()
{
    impl->bodyNode->start(impl->io, impl->maxPublishRate);
    return true;
}


void BodyPublisherItem::input()
{
    impl->bodyNode->input();
}


bool BodyPublisherItem::control()
{
    impl->bodyNode->control();
    return true;
}


void BodyPublisherItem::output()
{
    impl->bodyNode->output();
}


void BodyPublisherItem::stop()
{
    impl->bodyNode->stop();
    impl->io = nullptr;
}


void BodyPublisherItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Max publish rate"), impl->maxPublishRate, changeProperty(impl->maxPublishRate));
}


bool BodyPublisherItem::store(Archive& archive)
{
    archive.write("maxPublishRate", impl->maxPublishRate);
    return true;
}


bool BodyPublisherItem::restore(const Archive& archive)
{
    archive.read("maxPublishRate", impl->maxPublishRate);
    return true;
}


BodyNode::BodyNode(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      timeBar(TimeBar::instance())
{
    string name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    jointStatePublisher = rosNode->advertise<sensor_msgs::JointState>("joint_state", 1000);
    startToPublishKinematicStateChangeOnGUI();

    auto body = bodyItem->body();
    DeviceList<> devices = body->devices();

    cameras.assign(devices.extract<Camera>());
    image_transport::ImageTransport it(*rosNode);
    cameraImagePublishers.resize(cameras.size());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        cameraImagePublishers[i] = it.advertise(camera->name() + "/image", 1);
    }
}


void BodyNode::start(ControllerIO* io, double maxPublishRate)
{
    ioBody = io->body();
    time = 0.0;
    minPublishCycle = maxPublishRate > 0.0 ? (1.0 / maxPublishRate) : 0.0;
    timeToPublishNext = minPublishCycle;
    timeStep = io->timeStep();

    stopToPublishKinematicStateChangeOnGUI();
    initializeJointState(ioBody);

    sensorConnections.disconnect();
    DeviceList<> devices = ioBody->devices();

    cameras.assign(devices.extract<Camera>());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, i](){ publishCameraImage(i); }));
    }
}


void BodyNode::input()
{
    timeToPublishNext += timeStep;
    if(timeToPublishNext > minPublishCycle){
        publishJointState(ioBody, time);
        timeToPublishNext -= minPublishCycle;
    }
}


void BodyNode::control()
{
    time += timeStep;
}


void BodyNode::output()
{

}


void BodyNode::stop()
{
    stopToPublishKinematicStateChangeOnGUI();
    sensorConnections.disconnect();
}


void BodyNode::startToPublishKinematicStateChangeOnGUI()
{
    if(jointStatePublisher){
        auto body = bodyItem->body();
        initializeJointState(body);
        publishJointState(body, timeBar->time());
        connectionOfKinematicStateChange.reset(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ publishJointState(bodyItem->body(), timeBar->time()); }));
    }
}


void BodyNode::stopToPublishKinematicStateChangeOnGUI()
{
    connectionOfKinematicStateChange.disconnect();
}

        
void BodyNode::initializeJointState(Body* body)
{
    const int n = body->numJoints();
    jointState.name.resize(n);
    jointState.position.resize(n);
    jointState.velocity.resize(n);
    jointState.effort.resize(n);
    for(int i=0; i < n; ++i){
        jointState.name[i] = body->joint(i)->name();
    }
}
    

void BodyNode::publishJointState(Body* body, double time)
{
    jointState.header.stamp.fromSec(time);

    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        jointState.position[i] = joint->q();
        jointState.velocity[i] = joint->dq();
        jointState.effort[i] = joint->u();
    }

    jointStatePublisher.publish(jointState);
}


void BodyNode::publishCameraImage(int index)
{
    auto camera = cameras[index];
    sensor_msgs::Image image;
    image.header.stamp.fromSec(time);
    image.header.frame_id = camera->name();
    image.height = camera->image().height();
    image.width = camera->image().width();
    if(camera->image().numComponents() == 3){
        image.encoding = sensor_msgs::image_encodings::RGB8;
    } else if (camera->image().numComponents() == 1){
        image.encoding = sensor_msgs::image_encodings::MONO8;
    } else {
        ROS_WARN("unsupported image component number: %i", camera->image().numComponents());
    }
    image.is_bigendian = 0;
    image.step = camera->image().width() * camera->image().numComponents();
    image.data.resize(image.step * image.height);
    std::memcpy(&(image.data[0]), &(camera->image().pixels()[0]), image.step * image.height);
    cameraImagePublishers[index].publish(image);
}
