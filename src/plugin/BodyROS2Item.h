#ifndef CNOID_ROS2_PLUGIN_BODY_ROS2_ITEM_H
#define CNOID_ROS2_PLUGIN_BODY_ROS2_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <cnoid/Device>
#include <cnoid/DeviceList>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Archive>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/msg/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <image_transport/image_transport.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyROS2Item : public ControllerItem, rclcpp::Node
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyROS2Item();
    BodyROS2Item(const BodyROS2Item& org);
    virtual ~BodyROS2Item();
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
    sensor_msgs::msg::JointState joint_state_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
    double joint_state_update_rate_;
    double joint_state_update_period_;
    double joint_state_last_update_;

    ControllerIO* io;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;

//    image_transport::ImageTransport image_transport_;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> force_sensor_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> rate_gyro_sensor_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> accel_sensor_publishers_;
    std::vector<std::shared_ptr<image_transport::CameraPublisher>> vision_sensor_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> range_vision_sensor_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> range_sensor_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr> range_sensor_pc_publishers_;

    std::vector<std::shared_ptr<rclcpp::ServiceBase>> force_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> rate_gyro_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> accel_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> vision_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> range_vision_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> range_sensor_switch_servers_;
    std::vector<std::shared_ptr<rclcpp::ServiceBase>> range_sensor_pc_switch_servers_;

    void updateForceSensor(ForceSensor* sensor, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher);
    void updateRateGyroSensor(RateGyroSensor* sensor, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher);
    void updateAccelSensor(AccelerationSensor* sensor, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher);
    void updateVisionSensor(Camera* sensor, std::shared_ptr<image_transport::CameraPublisher> publisher);
    void updateRangeVisionSensor(RangeCamera* sensor, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    void updateRangeSensor(RangeSensor* sensor, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher);
    void update3DRangeSensor(RangeSensor* sensor, rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher);

    void switchDevice(std_srvs::srv::SetBool::Request::ConstSharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response,
                      Device* sensor);
    builtin_interfaces::msg::Time getStampMsgFromSec(double sec);

    /**
      @brief Stop publish.
      This method call from BodyROS2Item::stop.
     */
//    void stop_publish();
};

typedef ref_ptr<BodyROS2Item> BodyROS2ItemPtr;
}

#endif
