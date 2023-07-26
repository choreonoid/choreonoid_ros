#include "BodyROSItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud2.h>
#ifdef CNOID_ROS_PLUGIN_USE_POINTCLOUD1
#include <sensor_msgs/PointCloud.h>
typedef sensor_msgs::PointCloud PointCloudTypeForRangeSensor;
#else
typedef sensor_msgs::PointCloud2 PointCloudTypeForRangeSensor;
#endif

#include <fmt/format.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "gettext.h"

using namespace cnoid;
using fmt::format;


void BodyROSItem::initializeClass(ExtensionManager* ext)
{ 
    ext->itemManager().registerClass<BodyROSItem>(N_("BodyROSItem"));
    ext->itemManager().addCreationPanel<BodyROSItem>();
}


BodyROSItem::BodyROSItem()
    : os(MessageView::instance()->cout())
{
    io = nullptr;
    jointStateUpdateRate = 100.0;
}


BodyROSItem::BodyROSItem(const BodyROSItem& org)
    : ControllerItem(org)
    , os(MessageView::instance()->cout())
{
    io = nullptr;
    jointStateUpdateRate = 100.0;
}


BodyROSItem::~BodyROSItem()
{
    stop();
}


Item* BodyROSItem::doDuplicate() const
{
    return new BodyROSItem(*this);
}


bool BodyROSItem::store(Archive& archive)
{
    archive.write("body_ros_version", 0);
    archive.write("joint_state_update_rate", jointStateUpdateRate);

    return true;
}


bool BodyROSItem::restore(const Archive& archive)
{
    archive.read({ "joint_state_update_rate", "jointStateUpdateRate" }, jointStateUpdateRate);
    return true;
}


void BodyROSItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(2).min(0.0)("Update rate", jointStateUpdateRate, changeProperty(jointStateUpdateRate));
}


bool BodyROSItem::initialize(ControllerIO* io)
{
    if (!io->body()) {
        MessageView::instance()->putln(
            format(_("BodyROSItem \"{0}\" is invalid because it is not assigned to a body."), displayName()),
            MessageView::WARNING);
        return false;
    }

    this->io = io;
    simulationBody = io->body();
    timeStep_ = io->worldTimeStep();
    controlTime_ = io->currentTime();

    return true;
}


bool BodyROSItem::start()
{
    // buffer of preserve currently state of joints.
    joint_state_.header.stamp.fromSec(controlTime_);
    joint_state_.name.resize(body()->numAllJoints());
    joint_state_.position.resize(body()->numAllJoints());
    joint_state_.velocity.resize(body()->numAllJoints());
    joint_state_.effort.resize(body()->numAllJoints());

    // preserve initial state of joints.
    for (size_t i = 0; i < body()->numAllJoints(); i++) {
        Link* joint = body()->joint(i);

        joint_state_.name[i]     = joint->jointName();
        joint_state_.position[i] = joint->q();
        joint_state_.velocity[i] = joint->dq();
        joint_state_.effort[i]   = joint->u();
    }

    std::string name = simulationBody->name();
    std::replace(name.begin(), name.end(), '-', '_');
    rosNode.reset(new ros::NodeHandle(name));
    createSensors(simulationBody);

    jointStatePublisher     = rosNode->advertise<sensor_msgs::JointState>("joint_states", 1000);
    jointStateUpdatePeriod = 1.0 / jointStateUpdateRate;
    jointStateLastUpdate   = io->currentTime();
    ROS_DEBUG("Joint state update rate %f", jointStateUpdateRate);

    return true;
}


void BodyROSItem::createSensors(BodyPtr body)
{
    DeviceList<> devices = body->devices();

    forceSensors_.assign(devices.extract<ForceSensor>());
    gyroSensors_.assign(devices.extract<RateGyroSensor>());
    accelSensors_.assign(devices.extract<AccelerationSensor>());
    imus_.assign(devices.extract<Imu>());
    visionSensors_.assign(devices.extract<Camera>());
    rangeVisionPointCloudSensors_.clear();
    rangeVisionDepthImageSensors_.clear();
    rangeSensors_.assign(devices.extract<RangeSensor>());

    for (size_t i=0; i < visionSensors_.size(); ++i) {
        if (Camera* sensor = visionSensors_[i]) {
            RangeCamera* camera = dynamic_cast<RangeCamera*>(sensor);
            if (camera) {
                if (camera->isOrganized()) {
                    rangeVisionDepthImageSensors_.push_back(camera);
                } else {
                    rangeVisionPointCloudSensors_.push_back(camera);
                }
            }
        }
    }

    auto itr = visionSensors_.begin();
    while(itr != visionSensors_.end()){
        if (Camera* sensor = *itr){
            RangeCamera* camera = dynamic_cast<RangeCamera*>(sensor);
            if (sensor->imageType() == cnoid::Camera::NO_IMAGE && camera->isOrganized()) {
                itr = visionSensors_.erase(itr);
            }else{
                ++itr;
            }
        }else{
            ++itr;
        }
    }

    forceSensorPublishers.clear();
    forceSensorPublishers.reserve(forceSensors_.size());
    forceSensorSwitchServers.clear();
    forceSensorSwitchServers.reserve(forceSensors_.size());
    for (ForceSensorPtr sensor : forceSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const ros::Publisher publisher
            = rosNode->advertise<geometry_msgs::WrenchStamped>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateForceSensor(sensor, publisher);
        });
        forceSensorPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        forceSensorSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create force sensor %s", sensor->name().c_str());
    }

    rateGyroSensorPublishers.clear();
    rateGyroSensorPublishers.reserve(gyroSensors_.size());
    rateGyroSensorSwitchServers.clear();
    rateGyroSensorSwitchServers.reserve(gyroSensors_.size());
    for (RateGyroSensorPtr sensor : gyroSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const ros::Publisher publisher
            = rosNode->advertise<sensor_msgs::Imu>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateRateGyroSensor(sensor, publisher);
        });
        rateGyroSensorPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        rateGyroSensorSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create gyro sensor %s", sensor->name().c_str());
    }

    accelSensorPublishers.clear();
    accelSensorPublishers.reserve(accelSensors_.size());
    accelSensorSwitchServers.clear();
    accelSensorSwitchServers.reserve(accelSensors_.size());
    for (AccelerationSensorPtr sensor : accelSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const ros::Publisher publisher
            = rosNode->advertise<sensor_msgs::Imu>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateAccelSensor(sensor, publisher);
        });
        accelSensorPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        accelSensorSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create accel sensor %s", sensor->name().c_str());
    }

    imuPublishers.clear();
    imuPublishers.reserve(imus_.size());
    imuSwitchServers.clear();
    imuSwitchServers.reserve(imus_.size());
    for (ImuPtr sensor : imus_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const ros::Publisher publisher
            = rosNode->advertise<sensor_msgs::Imu>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateImu(sensor, publisher);
        });
        accelSensorPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        accelSensorSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create IMU %s", sensor->name().c_str());
    }

    image_transport::ImageTransport it(*rosNode);
    visionSensorPublishers.clear();
    visionSensorPublishers.reserve(visionSensors_.size());
    visionSensorSwitchServers.clear();
    visionSensorSwitchServers.reserve(visionSensors_.size());
    for (CameraPtr sensor : visionSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const image_transport::CameraPublisher publisher
            = it.advertiseCamera(name + "/color/image_raw", 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateVisionSensor(sensor, publisher);
        });
        visionSensorPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        visionSensorSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create RGB camera %s (%f Hz)",
                 sensor->name().c_str(), sensor->frameRate());
    }

    rangeVisionSensorPointCloudPublishers.clear();
    rangeVisionSensorPointCloudPublishers.reserve(rangeVisionPointCloudSensors_.size());
    rangeVisionSensorPointCloudSwitchServers.clear();
    rangeVisionSensorPointCloudSwitchServers.reserve(rangeVisionPointCloudSensors_.size());
    for (RangeCameraPtr sensor : rangeVisionPointCloudSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const ros::Publisher publisher =
            rosNode->advertise<sensor_msgs::PointCloud2>(name + "/point_cloud", 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateRangeVisionSensorPointCloud(sensor, publisher);
        });
        rangeVisionSensorPointCloudPublishers.push_back(publisher);
        boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
            = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
        rangeVisionSensorPointCloudSwitchServers.push_back(
            rosNode->advertiseService(name + "/set_enabled", requestCallback));
        ROS_INFO("Create point cloud camera %s (%f Hz)", sensor->name().c_str(), sensor->frameRate());

    }

    rangeVisionSensorDepthImagePublishers.clear();
    rangeVisionSensorDepthImagePublishers.reserve(rangeVisionDepthImageSensors_.size());
    rangeVisionSensorDepthImageSwitchServers.clear();
    rangeVisionSensorDepthImageSwitchServers.reserve(rangeVisionDepthImageSensors_.size());
    for (RangeCameraPtr sensor : rangeVisionDepthImageSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        const image_transport::CameraPublisher publisher 
            = it.advertiseCamera(name + "/depth/image_raw", 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateRangeVisionSensorDepthImage(sensor, publisher);
        });
        rangeVisionSensorDepthImagePublishers.push_back(publisher);
        // adds a server only for the camera whose type is COLOR_DEPTH.
        // Without this exception, a new service server may be a duplicate
        // of one added to 'visionSensorSwitchServers'.
        if (sensor->imageType() == Camera::NO_IMAGE) {
            boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
                = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
            rangeVisionSensorDepthImageSwitchServers.push_back(
                rosNode->advertiseService(name + "/set_enabled", requestCallback));
            ROS_INFO("Create depth camera %s (%f Hz)", sensor->name().c_str(), sensor->frameRate());
        } else {
            ROS_INFO("Create RGBD camera %s (%f Hz)", sensor->name().c_str(), sensor->frameRate());
        }
    }

#ifdef CNOID_ROS_PLUGIN_USE_POINTCLOUD1
    typedef sensor_msgs::PointCloud PointCloudTypeForRangeSensor;
#else
    typedef sensor_msgs::PointCloud2 PointCloudTypeForRangeSensor;
#endif

    rangeSensorPublishers.clear();
    rangeSensorPublishers.reserve(rangeSensors_.size());
    rangeSensorSwitchServers.clear();
    rangeSensorSwitchServers.reserve(rangeSensors_.size());
    rangeSensorPcPublishers.clear();
    rangeSensorPcPublishers.reserve(rangeSensors_.size());
    rangeSensorPcSwitchServers.clear();
    rangeSensorPcSwitchServers.reserve(rangeSensors_.size());
    for (RangeSensorPtr sensor : rangeSensors_) {
        if (sensor->numPitchSamples() > 1) {
            std::string name = sensor->name();
            std::replace(name.begin(), name.end(), '-', '_');
            const ros::Publisher publisher =
                rosNode->advertise<PointCloudTypeForRangeSensor>(name + "/point_cloud", 1);
            sensor->sigStateChanged().connect([this, sensor, publisher]() {
                update3DRangeSensor(sensor, publisher);
            });
            rangeSensorPcPublishers.push_back(publisher);
            boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
                = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
            rangeSensorPcSwitchServers.push_back(
                rosNode->advertiseService(name + "/set_enabled", requestCallback));
            ROS_INFO("Create 3d range sensor %s (%f Hz)",
                     sensor->name().c_str(), sensor->scanRate());
        } else {
            std::string name = sensor->name();
            std::replace(name.begin(), name.end(), '-', '_');
            const ros::Publisher publisher
                = rosNode->advertise<sensor_msgs::LaserScan>(name + "/scan", 1);
            sensor->sigStateChanged().connect([this, sensor, publisher]() {
                updateRangeSensor(sensor, publisher);
            });
            rangeSensorPublishers.push_back(publisher);
            boost::function<bool (std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&)> requestCallback
                = boost::bind(&BodyROSItem::switchDevice, this, _1, _2, sensor);
            rangeSensorSwitchServers.push_back(
                rosNode->advertiseService(name + "/set_enabled", requestCallback));
            ROS_INFO("Create 2d range sensor %s (%f Hz)",
                     sensor->name().c_str(), sensor->scanRate());
        }
    }
}


bool BodyROSItem::control()
{
    controlTime_ = io->currentTime();
    double updateSince = controlTime_ - jointStateLastUpdate;

    if (updateSince > jointStateUpdatePeriod) {
        // publish current joint states
        joint_state_.header.stamp.fromSec(controlTime_);

        for (int i = 0; i < body()->numAllJoints(); i++) {
            Link* joint = body()->joint(i);

            joint_state_.position[i] = joint->q();
            joint_state_.velocity[i] = joint->dq();
            joint_state_.effort[i]   = joint->u();
        }

        jointStatePublisher.publish(joint_state_);
        jointStateLastUpdate += jointStateUpdatePeriod;
    }

    return true;
}


void BodyROSItem::updateForceSensor
(const ForceSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    geometry_msgs::WrenchStamped force;
    force.header.stamp.fromSec(io->currentTime());
    force.header.frame_id = sensor->name();
    force.wrench.force.x = sensor->F()[0] / 1000.0;
    force.wrench.force.y = sensor->F()[1] / 1000.0;
    force.wrench.force.z = sensor->F()[2] / 1000.0;
    force.wrench.torque.x = sensor->F()[3] / 1000.0;
    force.wrench.torque.y = sensor->F()[4] / 1000.0;
    force.wrench.torque.z = sensor->F()[5] / 1000.0;
    publisher.publish(force);
}


void BodyROSItem::updateRateGyroSensor
(const RateGyroSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::Imu gyro;
    gyro.header.stamp.fromSec(io->currentTime());
    gyro.header.frame_id = sensor->name();
    gyro.angular_velocity.x = sensor->w()[0];
    gyro.angular_velocity.y = sensor->w()[1];
    gyro.angular_velocity.z = sensor->w()[2];
    publisher.publish(gyro);
}


void BodyROSItem::updateAccelSensor
(const AccelerationSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::Imu accel;
    accel.header.stamp.fromSec(io->currentTime());
    accel.header.frame_id = sensor->name();
    accel.linear_acceleration.x = sensor->dv()[0];
    accel.linear_acceleration.y = sensor->dv()[1];
    accel.linear_acceleration.z = sensor->dv()[2];
    publisher.publish(accel);
}


void BodyROSItem::updateImu
(const ImuPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::Imu imu;
    imu.header.stamp.fromSec(io->currentTime());
    imu.header.frame_id = sensor->name();
    imu.angular_velocity.x = sensor->w()[0];
    imu.angular_velocity.y = sensor->w()[1];
    imu.angular_velocity.z = sensor->w()[2];
    imu.linear_acceleration.x = sensor->dv()[0];
    imu.linear_acceleration.y = sensor->dv()[1];
    imu.linear_acceleration.z = sensor->dv()[2];
    publisher.publish(imu);
}


void BodyROSItem::updateVisionSensor
(const CameraPtr& sensor, const image_transport::CameraPublisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::Image vision;
    vision.header.stamp.fromSec(io->currentTime());
    vision.header.frame_id = sensor->name();
    vision.height = sensor->image().height();
    vision.width = sensor->image().width();
    if (sensor->image().numComponents() == 3)
        vision.encoding = sensor_msgs::image_encodings::RGB8;
    else if (sensor->image().numComponents() == 1)
        vision.encoding = sensor_msgs::image_encodings::MONO8;
    else {
        ROS_WARN("unsupported image component number: %i", sensor->image().numComponents());
    }
    vision.is_bigendian = 0;
    vision.step = sensor->image().width() * sensor->image().numComponents();
    vision.data.resize(vision.step * vision.height);
    std::memcpy(&(vision.data[0]), &(sensor->image().pixels()[0]), vision.step * vision.height);

    sensor_msgs::CameraInfo info;
    info.header = vision.header;
    info.width = vision.width;
    info.height = vision.height;
    info.distortion_model = "plumb_bob";
    info.D.resize(5, 0.0);

    const double fov2 = sensor->fieldOfView() / 2.0;
    const double minLength = std::min(info.width, info.height);
    const double focalLength = minLength / 2.0 / tan(fov2);
    const double principalPointX = (info.width - 1.0) / 2.0;
    const double principalPointY = (info.height - 1.0) / 2.0;

    info.K.assign(0.0);
    info.K[0] = focalLength;
    info.K[2] = principalPointX;
    info.K[4] = focalLength;
    info.K[5] = principalPointY;

    info.P.assign(0.0);
    info.P[0] = focalLength;
    info.P[2] = principalPointX;
    info.P[5] = focalLength;
    info.P[6] = principalPointY;
    info.P[10] = 1.0;
    
    info.R.assign(0.0);
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;
    
    publisher.publish(vision, info);
}


void BodyROSItem::updateRangeVisionSensorPointCloud
(const RangeCameraPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::PointCloud2 range;
    range.header.stamp.fromSec(io->currentTime());
    range.header.frame_id = sensor->name();
    range.width = sensor->resolutionX();
    range.height = sensor->resolutionY();
    range.is_bigendian = false;
    range.is_dense = true;
    if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
        range.fields.resize(4);
        range.fields[3].name = "rgb";
        range.fields[3].offset = 12;
        range.fields[3].count = 1;
        range.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        /*
          range.fields[3].name = "r";
          range.fields[3].offset = 12;
          range.fields[3].datatype = sensor_msgs::PointField::UINT8;
          range.fields[3].count = 1;
          range.fields[4].name = "g";
          range.fields[4].offset = 13;
          range.fields[4].datatype = sensor_msgs::PointField::UINT8;
          range.fields[4].count = 1;
          range.fields[5].name = "b";
          range.fields[5].offset = 14;
          range.fields[5].datatype = sensor_msgs::PointField::UINT8;
          range.fields[5].count = 1;
        */
        range.point_step = 16;
    } else {
        range.fields.resize(3);
        range.point_step = 12;
    }
    range.row_step = range.point_step * range.width;
    range.fields[0].name = "x";
    range.fields[0].offset = 0;
    range.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[0].count = 4;
    range.fields[1].name = "y";
    range.fields[1].offset = 4;
    range.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[1].count = 4;
    range.fields[2].name = "z";
    range.fields[2].offset = 8;
    range.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[2].count = 4;
    const std::vector<Vector3f>& points = sensor->constPoints();
    const unsigned char* pixels = sensor->constImage().pixels();
    range.data.resize(points.size() * range.point_step);
    unsigned char* dst = (unsigned char*)&(range.data[0]);
    for (size_t j = 0; j < points.size(); ++j) {
        float x = points[j].x();
        float y = points[j].y();
        float z = points[j].z();
        std::memcpy(&dst[0], &x, 4);
        std::memcpy(&dst[4], &y, 4);
        std::memcpy(&dst[8], &z, 4);
        if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
            dst[14] = *pixels++;
            dst[13] = *pixels++;
            dst[12] = *pixels++;
            dst[15] = 0;
        }
        dst += range.point_step;
    }
    publisher.publish(range);
}

void BodyROSItem::updateRangeVisionSensorDepthImage
(const RangeCameraPtr& sensor, const image_transport::CameraPublisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers()==0){
        return;
    }
    sensor_msgs::Image vision;
    vision.header.stamp.fromSec(io->currentTime());
    vision.header.frame_id = sensor->name();
    vision.height = sensor->resolutionY();
    vision.width = sensor->resolutionX();
    vision.encoding = sensor_msgs::image_encodings::MONO16;

    vision.is_bigendian = 0;
    vision.step = sensor->resolutionX() * sizeof(uint16_t);
    vision.data.resize(vision.step * vision.height);

    uint16_t* dst = (uint16_t*)&(vision.data[0]);
    const std::vector<Vector3f>& points = sensor->constPoints();
    for (int y = 0; y < vision.height; ++y ) {
        for (int x = 0; x < vision.width; ++x ) {
            int idx = y * vision.width + x;
            dst[idx] = (uint16_t)(points[idx].z() * 1000);
        }
    }

    sensor_msgs::CameraInfo info;
    info.header = vision.header;
    info.width = vision.width;
    info.height = vision.height;
    info.distortion_model = "plumb_bob";
    info.D.resize(5, 0.0);

    const double fov2 = sensor->fieldOfView() / 2.0;
    const double minLength = std::min(info.width, info.height);
    const double focalLength = minLength / 2.0 / tan(fov2);
    const double principalPointX = (info.width - 1.0) / 2.0;
    const double principalPointY = (info.height - 1.0) / 2.0;

    info.K.assign(0.0);
    info.K[0] = focalLength;
    info.K[2] = principalPointX;
    info.K[4] = focalLength;
    info.K[5] = principalPointY;

    info.P.assign(0.0);
    info.P[0] = focalLength;
    info.P[2] = principalPointX;
    info.P[5] = focalLength;
    info.P[6] = principalPointY;
    info.P[10] = 1.0;
    
    info.R.assign(0.0);
    info.R[0] = 1.0;
    info.R[4] = 1.0;
    info.R[8] = 1.0;

    publisher.publish(vision, info);
}

void BodyROSItem::updateRangeSensor
(const RangeSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::LaserScan range;
    range.header.stamp.fromSec(io->currentTime());
    range.header.frame_id = sensor->name();
    range.range_max = sensor->maxDistance();
    range.range_min = sensor->minDistance();
    if (sensor->yawRange() == 0.0) {
        range.angle_max = sensor->pitchRange()/2.0;
        range.angle_min = -sensor->pitchRange()/2.0;
        range.angle_increment = sensor->pitchStep();
    } else {
        range.angle_max = sensor->yawRange()/2.0;
        range.angle_min = -sensor->yawRange()/2.0;
        range.angle_increment = sensor->yawStep();
    }
    range.ranges.resize(sensor->rangeData().size());
    //range.intensities.resize(sensor->rangeData().size());
    // for (size_t j = 0; j < sensor->rangeData().size(); ++j) {
    for (size_t j = 0; j < sensor->numYawSamples(); ++j) {
        range.ranges[j] = sensor->rangeData()[j];
        //range.intensities[j] = -900000;
    }
    publisher.publish(range);
}


#ifndef CNOID_ROS_PLUGIN_USE_POINTCLOUD1
void BodyROSItem::update3DRangeSensor
(const RangeSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::PointCloud2 range;
    // Header Info
    range.header.stamp.fromSec(io->currentTime());
    range.header.frame_id = sensor->name();

    // Calculate Point Cloud data
    const int numPitchSamples = sensor->numPitchSamples();
    const double pitchStep = sensor->pitchStep();
    const int numYawSamples = sensor->numYawSamples();
    const double yawStep = sensor->yawStep();

    range.height = numPitchSamples;
    range.width = numYawSamples;
    range.point_step = 12;
    range.row_step = range.width * range.point_step;
    range.fields.resize(3);
    range.fields[0].name = "x";
    range.fields[0].offset = 0;
    range.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[0].count = 4;
    range.fields[1].name = "y";
    range.fields[1].offset = 4;
    range.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[1].count = 4;
    range.fields[2].name = "z";
    range.fields[2].offset = 8;
    range.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    range.fields[2].count = 4;

    range.data.resize(numPitchSamples * numYawSamples * range.point_step);
    unsigned char* dst = (unsigned char*)&(range.data[0]);

    Matrix3f Ro;
    bool hasRo = !sensor->opticalFrameRotation().isIdentity();
    if(hasRo){
        Ro = sensor->opticalFrameRotation().cast<float>();
    }

    for(int pitchIndex = 0; pitchIndex < numPitchSamples; ++pitchIndex){
        const double pitchAngle =
            pitchIndex * pitchStep - sensor->pitchRange() / 2.0;
        const double cosPitchAngle = cos(pitchAngle);
        const double sinPitchAngle = sin(pitchAngle);
        const int srctop = pitchIndex * numYawSamples;

        for (int yawIndex = 0; yawIndex < numYawSamples; ++yawIndex) {
            const double distance = sensor->rangeData()[srctop + yawIndex];
            const double yawAngle = yawIndex * yawStep - sensor->yawRange() / 2.0;
            Vector3f p;
            p.x() = distance *  cosPitchAngle * sin(-yawAngle);
            p.y() = distance * sinPitchAngle;
            p.z() = -distance * cosPitchAngle * cos(-yawAngle);
            if(hasRo){
                p = Ro * p;
            }
            std::memcpy(&dst[0], &p.x(), 4);
            std::memcpy(&dst[4], &p.y(), 4);
            std::memcpy(&dst[8], &p.z(), 4);
            dst += range.point_step;
        }
    }

    publisher.publish(range);
}
#endif


#ifdef CNOID_ROS_PLUGIN_USE_POINTCLOUD1
void BodyROSItem::update3DRangeSensor
(const RangeSensorPtr& sensor, const ros::Publisher& publisher)
{
    if(!sensor->on()){
        return;
    }
    if(publisher.getNumSubscribers() == 0){
        return;
    }
    sensor_msgs::PointCloud range;
    // Header Info
    range.header.stamp.fromSec(io->currentTime());
    range.header.frame_id = sensor->name();

    // Calculate Point Cloud data
    const int numPitchSamples = sensor->numPitchSamples();
    const double pitchStep = sensor->pitchStep();
    const int numYawSamples = sensor->numYawSamples();
    const double yawStep = sensor->yawStep();

    for(int pitch=0; pitch < numPitchSamples; ++pitch){
        const double pitchAngle = pitch * pitchStep - sensor->pitchRange() / 2.0;
        const double cosPitchAngle = cos(pitchAngle);
        const int srctop = pitch * numYawSamples;

        for(int yaw=0; yaw < numYawSamples; ++yaw){
            const double distance = sensor->rangeData()[srctop + yaw];
            if(distance <= sensor->maxDistance()){
                double yawAngle = yaw * yawStep - sensor->yawRange() / 2.0;
                geometry_msgs::Point32 point;
                point.x = distance *  cosPitchAngle * sin(-yawAngle);
                point.y = distance * sin(pitchAngle);
                point.z = -distance * cosPitchAngle * cos(-yawAngle);
                range.points.push_back(point);
            }
        }
    }

    publisher.publish(range);
}
#endif


void BodyROSItem::input()
{

}


void BodyROSItem::output()
{

}


void BodyROSItem::stopPublishing()
{
    for (size_t i = 0; i < forceSensorPublishers.size(); ++i) {
        forceSensorPublishers[i].shutdown();
    }
    for (size_t i = 0; i < rateGyroSensorPublishers.size(); ++i) {
        rateGyroSensorPublishers[i].shutdown();
    }
    for (size_t i = 0; i < accelSensorPublishers.size(); ++i) {
        accelSensorPublishers[i].shutdown();
    }
    for (size_t i = 0; i < imuPublishers.size(); ++i) {
        imuPublishers[i].shutdown();
    }
    for (size_t i = 0; i < visionSensorPublishers.size(); ++i) {
        visionSensorPublishers[i].shutdown();
    }
    for (size_t i = 0; i < rangeVisionSensorPointCloudPublishers.size(); ++i) {
        rangeVisionSensorPointCloudPublishers[i].shutdown();
    }
    for (size_t i = 0; i < rangeVisionSensorDepthImagePublishers.size(); ++i) {
        rangeVisionSensorDepthImagePublishers[i].shutdown();
    }
    for (size_t i = 0; i < rangeSensorPublishers.size(); ++i) {
        rangeSensorPublishers[i].shutdown();
    }
    for (size_t i = 0; i < rangeSensorPcPublishers.size(); ++i) {
        rangeSensorPcPublishers[i].shutdown();
    }
}


bool BodyROSItem::switchDevice
(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response, Device* sensor)
{
    sensor->on(request.data);
    response.success = (request.data == sensor->on());
    return true;
}


void BodyROSItem::stop()
{
    if (ros::ok()) {
        stopPublishing();

        if (rosNode) {
            rosNode->shutdown();
        }
    }
}
