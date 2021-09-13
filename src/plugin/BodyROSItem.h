#ifndef CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Archive>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/SetBool.h>

#include <image_transport/image_transport.h>

#include <fstream>
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
    const DeviceList<Camera>& visionSensors() const { return visionSensors_; }
    const DeviceList<RangeCamera>& rangeVisionSensors() const { return rangeVisionSensors_; }
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
    DeviceList<Camera> visionSensors_;
    DeviceList<RangeCamera> rangeVisionSensors_;
    DeviceList<RangeSensor> rangeSensors_;
    double timeStep_;

    /* joint states */
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    double joint_state_update_rate_;
    double joint_state_update_period_;
    double joint_state_last_update_;

    ControllerIO* io;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    std::unique_ptr<ros::NodeHandle> rosnode_;
 
    std::vector<ros::Publisher> force_sensor_publishers_;
    std::vector<ros::Publisher> rate_gyro_sensor_publishers_;
    std::vector<ros::Publisher> accel_sensor_publishers_;
    std::vector<image_transport::Publisher> vision_sensor_publishers_;
    std::vector<ros::Publisher> range_vision_sensor_publishers_;
    std::vector<ros::Publisher> range_sensor_publishers_;
    std::vector<ros::Publisher> range_sensor_pc_publishers_;

    std::vector<ros::ServiceServer> force_sensor_switch_servers_;
    std::vector<ros::ServiceServer> rate_gyro_sensor_switch_servers_;
    std::vector<ros::ServiceServer> accel_sensor_switch_servers_;
    std::vector<ros::ServiceServer> vision_sensor_switch_servers_;
    std::vector<ros::ServiceServer> range_vision_sensor_switch_servers_;
    std::vector<ros::ServiceServer> range_sensor_switch_servers_;
    std::vector<ros::ServiceServer> range_sensor_pc_switch_servers_;

    void updateForceSensor(ForceSensor* sensor, ros::Publisher& publisher);
    void updateRateGyroSensor(RateGyroSensor* sensor, ros::Publisher& publisher);
    void updateAccelSensor(AccelerationSensor* sensor, ros::Publisher& publisher);
    void updateVisionSensor(Camera* sensor, image_transport::Publisher& publisher);
    void updateRangeVisionSensor(RangeCamera* sensor, ros::Publisher& publisher);
    void updateRangeSensor(RangeSensor* sensor, ros::Publisher& publisher);
    void update3DRangeSensor(RangeSensor* sensor, ros::Publisher& publisher);

    bool switchDevice(std_srvs::SetBoolRequest &request,
                      std_srvs::SetBoolResponse &response,
                      Device* sensor);

    /**
      @brief Stop publish.
      This method call from BodyROSItem::stop.
     */
    void stop_publish();
};

typedef ref_ptr<BodyROSItem> BodyROSItemPtr;
}

#endif
