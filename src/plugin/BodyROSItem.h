#ifndef CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/Imu>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Archive>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <image_transport/image_transport.h>

#include <ostream>
#include <memory>
#include <string>
#include <vector>

#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyROSItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyROSItem();
    BodyROSItem(const BodyROSItem& org);
    virtual ~BodyROSItem();
    void createSensors(BodyPtr body);

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override {
      return timeStep_;
    };
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    const Body* body() const { return simulationBody; };
    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& gyroSensors() const { return gyroSensors_; }
    const DeviceList<AccelerationSensor>& accelSensors() const { return accelSensors_; }
    const DeviceList<Imu>& imus() const { return imus_; }
    const DeviceList<Camera>& visionSensors() const { return visionSensors_; }
    const DeviceList<RangeCamera>& rangeVisionPointCloudSensors() const { return rangeVisionPointCloudSensors_; }
    const DeviceList<RangeCamera>& rangeVisionDepthImageSensors() const { return rangeVisionDepthImageSensors_; }
    const DeviceList<RangeSensor>& rangeSensors() const { return rangeSensors_; }

    double controlTime() const { return controlTime_; }

    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    BodyPtr simulationBody;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> gyroSensors_;
    DeviceList<AccelerationSensor> accelSensors_;
    DeviceList<Imu> imus_;
    DeviceList<Camera> visionSensors_;
    DeviceList<RangeCamera> rangeVisionPointCloudSensors_;
    DeviceList<RangeCamera> rangeVisionDepthImageSensors_;
    DeviceList<RangeSensor> rangeSensors_;
    double timeStep_;

    /* properties */
    bool jointStatePublication;
    double jointStateUpdateRate;

    /* joint states */
    sensor_msgs::JointState joint_state_;
    ros::Publisher jointStatePublisher;
    double jointStateUpdatePeriod;
    double jointStateLastUpdate;

    ControllerIO* io;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    std::unique_ptr<ros::NodeHandle> rosNode;

    std::vector<ros::Publisher> forceSensorPublishers;
    std::vector<ros::Publisher> rateGyroSensorPublishers;
    std::vector<ros::Publisher> accelSensorPublishers;
    std::vector<ros::Publisher> imuPublishers;
    std::vector<image_transport::CameraPublisher> visionSensorPublishers;
    // std::vector<ros::Publisher> rangeVisionSensorPublishers;
    std::vector<ros::Publisher> rangeVisionSensorPointCloudPublishers;
    std::vector<image_transport::CameraPublisher> rangeVisionSensorDepthImagePublishers;
    std::vector<ros::Publisher> rangeSensorPublishers;
    std::vector<ros::Publisher> rangeSensorPcPublishers;

    std::vector<ros::ServiceServer> forceSensorSwitchServers;
    std::vector<ros::ServiceServer> rateGyroSensorSwitchServers;
    std::vector<ros::ServiceServer> accelSensorSwitchServers;
    std::vector<ros::ServiceServer> imuSwitchServers;
    std::vector<ros::ServiceServer> visionSensorSwitchServers;
    std::vector<ros::ServiceServer> rangeVisionSensorPointCloudSwitchServers;
    std::vector<ros::ServiceServer> rangeVisionSensorDepthImageSwitchServers;
    std::vector<ros::ServiceServer> rangeSensorSwitchServers;
    std::vector<ros::ServiceServer> rangeSensorPcSwitchServers;

    void updateForceSensor(
        const ForceSensorPtr& sensor, const ros::Publisher& publisher);
    void updateRateGyroSensor(
        const RateGyroSensorPtr& sensor, const ros::Publisher& publisher);
    void updateAccelSensor(
        const AccelerationSensorPtr& sensor, const ros::Publisher& publisher);
    void updateImu(const ImuPtr& sensor, const ros::Publisher& publisher);
    void updateVisionSensor(
        const CameraPtr& sensor, const image_transport::CameraPublisher& publisher);
    void updateRangeVisionSensorPointCloud(
        const RangeCameraPtr& sensor, const ros::Publisher& publisher);
    void updateRangeVisionSensorDepthImage(
        const RangeCameraPtr& sensor, const image_transport::CameraPublisher& publisher);
    void updateRangeSensor(
        const RangeSensorPtr& sensor, const ros::Publisher& publisher);
    void update3DRangeSensor(
        const RangeSensorPtr& sensor, const ros::Publisher& publisher);

    bool switchDevice(
        std_srvs::SetBoolRequest &request, std_srvs::SetBoolResponse &response, Device* sensor);

    /**
      @brief Stop publishing.
      This method call from BodyROSItem::stop.
     */
    void stopPublishing();
};

typedef ref_ptr<BodyROSItem> BodyROSItemPtr;
}  // namespace cnoid

#endif
