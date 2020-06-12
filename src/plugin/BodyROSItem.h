#ifndef CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H_INCLUDED

#include <cnoid/ControllerItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Archive>
#include "exportdecl.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>

#include <image_transport/image_transport.h>

#include <vector>
#include <fstream>

namespace cnoid {

class CNOID_EXPORT BodyROSItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
    
    BodyROSItem();
    BodyROSItem(const BodyROSItem& org);
    virtual ~BodyROSItem();
    bool createSensors(BodyPtr body);
    
    virtual bool initialize(Target* target);
    virtual bool start();
    virtual double timeStep() const {
      return timeStep_;
    };
    virtual void input();
    virtual bool control();
    virtual void output();
    virtual void stop();
    
    const BodyPtr& body() const { return simulationBody; };
    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& gyroSensors() const { return gyroSensors_; }
    const DeviceList<AccelerationSensor>& accelSensors() const { return accelSensors_; }
    const DeviceList<Camera>& visionSensors() const { return visionSensors_; }
    const DeviceList<RangeCamera>& rangeVisionSensors() const { return rangeVisionSensors_; }
    const DeviceList<RangeSensor>& rangeSensors() const { return rangeSensors_; }
    
    double controlTime() const { return controlTime_; }
    
    void setModuleName(const std::string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    void doPutProperties(PutPropertyFunction& putProperty);

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

    const Target* controllerTarget;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
 
    std::vector<ros::Publisher> force_sensor_publishers_;
    std::vector<ros::Publisher> rate_gyro_sensor_publishers_;
    std::vector<ros::Publisher> accel_sensor_publishers_;
    std::vector<image_transport::Publisher> vision_sensor_publishers_;
    std::vector<ros::Publisher> range_vision_sensor_publishers_;
    std::vector<ros::Publisher> range_sensor_publishers_;
    std::vector<ros::Publisher> range_sensor_pc_publishers_;

    void updateForceSensor(ForceSensor* sensor, ros::Publisher& publisher);
    void updateRateGyroSensor(RateGyroSensor* sensor, ros::Publisher& publisher);
    void updateAccelSensor(AccelerationSensor* sensor, ros::Publisher& publisher);
    void updateVisionSensor(Camera* sensor, image_transport::Publisher& publisher);
    void updateRangeVisionSensor(RangeCamera* sensor, ros::Publisher& publisher);
    void updateRangeSensor(RangeSensor* sensor, ros::Publisher& publisher);
    void update3DRangeSensor(RangeSensor* sensor, ros::Publisher& publisher);

    /**
      @brief Stop publish.
      This method call from BodyROSItem::stop.
     */
    void stop_publish();
};

typedef ref_ptr<BodyROSItem> BodyROSItemPtr;
}

#endif
