#include "BodyROS2Item.h"
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/Link>
#include <cnoid/MessageView>
#include <cnoid/PutPropertyFunction>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef CNOID_ROS_PLUGIN_USE_POINTCLOUD1
#include <sensor_msgs/msg/point_cloud.hpp>
typedef sensor_msgs::msg::PointCloud PointCloudTypeForRangeSensor;
#else
typedef sensor_msgs::msg::PointCloud2 PointCloudTypeForRangeSensor;
#endif

#include "gettext.h"
#include <fmt/format.h>

using namespace cnoid;
using fmt::format;
using std::placeholders::_1;
using std::placeholders::_2;

void BodyROS2Item::initializeClass(ExtensionManager *ext)
{
    ext->itemManager().registerClass<BodyROS2Item>(N_("BodyROS2Item"));
    ext->itemManager().addCreationPanel<BodyROS2Item>();
}


BodyROS2Item::BodyROS2Item()
    : os(MessageView::instance()->cout())
{
    node_ = std::make_shared<rclcpp::Node>("choreonoid_body_ros2",
                                           rclcpp::NodeOptions());
    image_transport = std::make_shared<image_transport::ImageTransport>(node_);
    io = nullptr;
    jointStateUpdateRate = 100.0;
}


BodyROS2Item::BodyROS2Item(const BodyROS2Item &org)
    : ControllerItem(org)
    , os(MessageView::instance()->cout())
{
    node_ = std::make_shared<rclcpp::Node>("choreonoid_body_ros2",
                                           rclcpp::NodeOptions());
    image_transport = std::make_shared<image_transport::ImageTransport>(node_);
    io = nullptr;
    jointStateUpdateRate = 100.0;
}


BodyROS2Item::~BodyROS2Item()
{
    stop();
}


Item *BodyROS2Item::doDuplicate() const
{
    return new BodyROS2Item(*this);
}

bool BodyROS2Item::store(Archive &archive)
{
    archive.write("body_ros_version", 0);
    archive.write("joint_state_update_rate", jointStateUpdateRate);

    return true;
}


bool BodyROS2Item::restore(const Archive &archive)
{
    archive.read({"joint_state_update_rate", "jointStateUpdateRate"},
                 jointStateUpdateRate);
    return true;
}


void BodyROS2Item::doPutProperties(PutPropertyFunction &putProperty)
{
    putProperty.decimals(2).min(0.0)("Update rate",
                                     jointStateUpdateRate,
                                     changeProperty(jointStateUpdateRate));
}


bool BodyROS2Item::initialize(ControllerIO *io)
{
    if (!io->body()) {
        MessageView::instance()
            ->putln(format(_("BodyROS2Item \"{0}\" is invalid because it is "
                             "not assigned to a body."),
                           displayName()),
                    MessageView::WARNING);
        return false;
    }

    this->io = io;
    simulationBody = io->body();
    timeStep_ = io->worldTimeStep();
    controlTime_ = io->currentTime();

    return true;
}


bool BodyROS2Item::start()
{
    // buffer of preserve currently state of joints.
    joint_state_.header.stamp = getStampMsgFromSec(controlTime_);
    joint_state_.name.resize(body()->numAllJoints());
    joint_state_.position.resize(body()->numAllJoints());
    joint_state_.velocity.resize(body()->numAllJoints());
    joint_state_.effort.resize(body()->numAllJoints());

    // preserve initial state of joints.
    for (size_t i = 0; i < body()->numAllJoints(); i++) {
        Link *joint = body()->joint(i);

        joint_state_.name[i] = joint->name();
        joint_state_.position[i] = joint->q();
        joint_state_.velocity[i] = joint->dq();
        joint_state_.effort[i] = joint->u();
    }

    std::string name = simulationBody->name();
    std::replace(name.begin(), name.end(), '-', '_');
    // rosNode.reset(new ros::NodeHandle(name));
    createSensors(simulationBody);

    jointStatePublisher
        = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                1000);
    jointStateUpdatePeriod = 1.0 / jointStateUpdateRate;
    jointStateLastUpdate = io->currentTime();
    RCLCPP_DEBUG(node_->get_logger(),
                 "Joint state update rate %f",
                 jointStateUpdateRate);

    return true;
}


void BodyROS2Item::createSensors(BodyPtr body)
{
    using SetBoolCallback = std::function<
        void(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
             std::shared_ptr<std_srvs::srv::SetBool::Response>)>;

    DeviceList<> devices = body->devices();

    forceSensors_.assign(devices.extract<ForceSensor>());
    gyroSensors_.assign(devices.extract<RateGyroSensor>());
    accelSensors_.assign(devices.extract<AccelerationSensor>());
    visionSensors_.assign(devices.extract<Camera>());
    rangeVisionSensors_.assign(devices.extract<RangeCamera>());
    rangeSensors_.assign(devices.extract<RangeSensor>());

    for (size_t i = 0; i < visionSensors_.size(); ++i) {
        if (Camera *sensor = visionSensors_[i]) {
            RangeCamera *camera = dynamic_cast<RangeCamera *>(sensor);
            if (camera) {
                rangeVisionSensors_.push_back(camera);
            }
        }
    }

    forceSensorPublishers.clear();
    forceSensorPublishers.reserve(forceSensors_.size());
    forceSensorSwitchServers.clear();
    forceSensorSwitchServers.reserve(forceSensors_.size());
    for (auto sensor : forceSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        auto publisher
            = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(name,
                                                                         1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateForceSensor(sensor, publisher);
        });
        forceSensorPublishers.push_back(publisher);
        SetBoolCallback requestCallback = std::bind(&BodyROS2Item::switchDevice,
                                                    this,
                                                    _1,
                                                    _2,
                                                    sensor);
        forceSensorSwitchServers.push_back(
            node_->create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                          requestCallback));
        RCLCPP_INFO(node_->get_logger(),
                    "Create force sensor %s",
                    sensor->name().c_str());
    }

    rateGyroSensorPublishers.clear();
    rateGyroSensorPublishers.reserve(gyroSensors_.size());
    rateGyroSensorSwitchServers.clear();
    rateGyroSensorSwitchServers.reserve(gyroSensors_.size());
    for (auto sensor : gyroSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        auto publisher = node_->create_publisher<sensor_msgs::msg::Imu>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateRateGyroSensor(sensor, publisher);
        });
        rateGyroSensorPublishers.push_back(publisher);
        SetBoolCallback requestCallback = std::bind(&BodyROS2Item::switchDevice,
                                                    this,
                                                    _1,
                                                    _2,
                                                    sensor);
        rateGyroSensorSwitchServers.push_back(
            node_->create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                          requestCallback));
        RCLCPP_INFO(node_->get_logger(),
                    "Create gyro sensor %s",
                    sensor->name().c_str());
    }

    accelSensorPublishers.clear();
    accelSensorPublishers.reserve(accelSensors_.size());
    accelSensorSwitchServers.clear();
    accelSensorSwitchServers.reserve(accelSensors_.size());
    for (auto sensor : accelSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        auto publisher = node_->create_publisher<sensor_msgs::msg::Imu>(name, 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateAccelSensor(sensor, publisher);
        });
        accelSensorPublishers.push_back(publisher);
        SetBoolCallback requestCallback = std::bind(&BodyROS2Item::switchDevice,
                                                    this,
                                                    _1,
                                                    _2,
                                                    sensor);
        accelSensorSwitchServers.push_back(
            node_->create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                          requestCallback));
        RCLCPP_INFO(node_->get_logger(),
                    "Create accel sensor %s",
                    sensor->name().c_str());
    }

    visionSensorPublishers.clear();
    visionSensorPublishers.reserve(visionSensors_.size());
    visionSensorSwitchServers.clear();
    visionSensorSwitchServers.reserve(visionSensors_.size());
    for (CameraPtr sensor : visionSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');

        visionSensorPublishers.push_back(image_transport->advertise(name, 1));
        auto & publisher = visionSensorPublishers.back();
        sensor->sigStateChanged().connect([this, sensor, &publisher]() {
            updateVisionSensor(sensor, publisher);
        });
        SetBoolCallback requestCallback = std::bind(&BodyROS2Item::switchDevice,
                                                    this,
                                                    _1,
                                                    _2,
                                                    sensor);
        visionSensorSwitchServers.push_back(
            node_->create_service<std_srvs::srv::SetBool>(name + "/set_enabled",
                                                          requestCallback));
        RCLCPP_INFO(node_->get_logger(),
                    "Create RGB camera %s (%f Hz)",
                    sensor->name().c_str(),
                    sensor->frameRate());
    }

    rangeVisionSensorPublishers.clear();
    rangeVisionSensorPublishers.reserve(rangeVisionSensors_.size());
    rangeVisionSensorSwitchServers.clear();
    rangeVisionSensorSwitchServers.reserve(rangeVisionSensors_.size());
    for (auto sensor : rangeVisionSensors_) {
        std::string name = sensor->name();
        std::replace(name.begin(), name.end(), '-', '_');
        auto publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            name + "/point_cloud", 1);
        sensor->sigStateChanged().connect([this, sensor, publisher]() {
            updateRangeVisionSensor(sensor, publisher);
        });
        rangeVisionSensorPublishers.push_back(publisher);
        // adds a server only for the camera whose type is COLOR_DEPTH or POINT_CLOUD.
        // Without this exception, a new service server may be a duplicate
        // of one added to 'visionSensorSwitchServers'.
        if (sensor->imageType() == Camera::NO_IMAGE) {
            SetBoolCallback requestCallback
                = std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
            rangeVisionSensorSwitchServers.push_back(
                node_->create_service<std_srvs::srv::SetBool>(
                    name + "/set_enabled", requestCallback));
            RCLCPP_INFO(node_->get_logger(),
                        "Create depth camera %s (%f Hz)",
                        sensor->name().c_str(),
                        sensor->frameRate());
        } else {
            RCLCPP_INFO(node_->get_logger(),
                        "Create RGBD camera %s (%f Hz)",
                        sensor->name().c_str(),
                        sensor->frameRate());
        }
    }

    rangeSensorPublishers.clear();
    rangeSensorPublishers.reserve(rangeSensors_.size());
    rangeSensorSwitchServers.clear();
    rangeSensorSwitchServers.reserve(rangeSensors_.size());
    rangeSensorPcPublishers.clear();
    rangeSensorPcPublishers.reserve(rangeSensors_.size());
    rangeSensorPcSwitchServers.clear();
    rangeSensorPcSwitchServers.reserve(rangeSensors_.size());
    for (auto sensor : rangeSensors_) {
        if (sensor->numPitchSamples() > 1) {
            std::string name = sensor->name();
            std::replace(name.begin(), name.end(), '-', '_');
            auto pc_publisher = node_->create_publisher<
                sensor_msgs::msg::PointCloud>(name + "/point_cloud", 1);
            sensor->sigStateChanged().connect([this, sensor, pc_publisher]() {
                update3DRangeSensor(sensor, pc_publisher);
            });
            rangeSensorPcPublishers.push_back(pc_publisher);
            SetBoolCallback requestCallback
                = std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
            rangeSensorPcSwitchServers.push_back(
                node_->create_service<std_srvs::srv::SetBool>(
                    name + "/set_enabled", requestCallback));
            RCLCPP_DEBUG(node_->get_logger(),
                         "Create 3d range sensor %s (%f Hz)",
                         sensor->name().c_str(),
                         sensor->scanRate());
        } else {
            std::string name = sensor->name();
            std::replace(name.begin(), name.end(), '-', '_');
            auto publisher = node_->create_publisher<
                sensor_msgs::msg::LaserScan>(name + "/scan", 1);
            sensor->sigStateChanged().connect([this, sensor, publisher]() {
                updateRangeSensor(sensor, publisher);
            });
            rangeSensorPublishers.push_back(publisher);
            SetBoolCallback requestCallback
                = std::bind(&BodyROS2Item::switchDevice, this, _1, _2, sensor);
            rangeSensorSwitchServers.push_back(
                node_->create_service<std_srvs::srv::SetBool>(
                    name + "/set_enabled", requestCallback));
            RCLCPP_DEBUG(node_->get_logger(),
                         "Create 2d range sensor %s (%f Hz)",
                         sensor->name().c_str(),
                         sensor->scanRate());
        }
    }
}

bool BodyROS2Item::control()
{
    controlTime_ = io->currentTime();
    double updateSince = controlTime_ - jointStateLastUpdate;

    if (updateSince > jointStateUpdatePeriod) {
        // publish current joint states
        joint_state_.header.stamp = getStampMsgFromSec(controlTime_);

        for (int i = 0; i < body()->numAllJoints(); i++) {
            Link *joint = body()->joint(i);

            joint_state_.position[i] = joint->q();
            joint_state_.velocity[i] = joint->dq();
            joint_state_.effort[i] = joint->u();
        }

        jointStatePublisher->publish(joint_state_);
        jointStateLastUpdate += jointStateUpdatePeriod;
    }

    rclcpp::spin_some(node_);
    return true;
}


void BodyROS2Item::updateForceSensor(
    ForceSensor *sensor,
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    geometry_msgs::msg::WrenchStamped force;
    force.header.stamp = getStampMsgFromSec(io->currentTime());
    force.header.frame_id = sensor->name();
    force.wrench.force.x = sensor->F()[0] / 1000.0;
    force.wrench.force.y = sensor->F()[1] / 1000.0;
    force.wrench.force.z = sensor->F()[2] / 1000.0;
    force.wrench.torque.x = sensor->F()[3] / 1000.0;
    force.wrench.torque.y = sensor->F()[4] / 1000.0;
    force.wrench.torque.z = sensor->F()[5] / 1000.0;
    publisher->publish(force);
}


void BodyROS2Item::updateRateGyroSensor(
    RateGyroSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    sensor_msgs::msg::Imu gyro;
    gyro.header.stamp = getStampMsgFromSec(io->currentTime());
    gyro.header.frame_id = sensor->name();
    gyro.angular_velocity.x = sensor->w()[0];
    gyro.angular_velocity.y = sensor->w()[1];
    gyro.angular_velocity.z = sensor->w()[2];
    publisher->publish(gyro);
}


void BodyROS2Item::updateAccelSensor(
    AccelerationSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    sensor_msgs::msg::Imu accel;
    accel.header.stamp = getStampMsgFromSec(io->currentTime());
    accel.header.frame_id = sensor->name();
    accel.linear_acceleration.x = sensor->dv()[0] / 10.0;
    accel.linear_acceleration.y = sensor->dv()[1] / 10.0;
    accel.linear_acceleration.z = sensor->dv()[2] / 10.0;
    publisher->publish(accel);
}


void BodyROS2Item::updateVisionSensor(
    Camera *sensor,
    image_transport::Publisher & publisher)
{
    if (!sensor->on()) {
        return;
    }

    sensor_msgs::msg::Image vision;
    vision.header.stamp = getStampMsgFromSec(io->currentTime());
    vision.header.frame_id = sensor->name();
    vision.height = sensor->image().height();
    vision.width = sensor->image().width();
    if (sensor->image().numComponents() == 3)
        vision.encoding = "rgb8";
    else if (sensor->image().numComponents() == 1)
        vision.encoding = "mono8";
    else {
        RCLCPP_WARN(node_->get_logger(),
                    "unsupported image component number: %i",
                    sensor->image().numComponents());
    }
    vision.is_bigendian = 0;
    vision.step = sensor->image().width() * sensor->image().numComponents();
    vision.data.resize(vision.step * vision.height);
    std::memcpy(&(vision.data[0]),
                &(sensor->image().pixels()[0]),
                vision.step * vision.height);
    publisher.publish(vision);
}


void BodyROS2Item::updateRangeVisionSensor(
    RangeCamera *sensor,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    sensor_msgs::msg::PointCloud2 range;
    range.header.stamp = getStampMsgFromSec(io->currentTime());
    range.header.frame_id = sensor->name();
    range.width = sensor->resolutionX();
    range.height = sensor->resolutionY();
    range.is_bigendian = false;
    range.is_dense = true;
    range.row_step = range.point_step * range.width;
    if (sensor->imageType() == cnoid::Camera::COLOR_IMAGE) {
        range.fields.resize(6);
        range.fields[3].name = "rgb";
        range.fields[3].offset = 12;
        range.fields[3].count = 1;
        range.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
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
    range.fields[0].name = "x";
    range.fields[0].offset = 0;
    range.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    range.fields[0].count = 4;
    range.fields[1].name = "y";
    range.fields[1].offset = 4;
    range.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    range.fields[1].count = 4;
    range.fields[2].name = "z";
    range.fields[2].offset = 8;
    range.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    range.fields[2].count = 4;
    const std::vector<Vector3f> &points = sensor->constPoints();
    const unsigned char *pixels = sensor->constImage().pixels();
    range.data.resize(points.size() * range.point_step);
    unsigned char *dst = (unsigned char *) &(range.data[0]);
    for (size_t j = 0; j < points.size(); ++j) {
        float x = points[j].x();
        float y = -points[j].y();
        float z = -points[j].z();
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
    publisher->publish(range);
}


void BodyROS2Item::updateRangeSensor(
    RangeSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    sensor_msgs::msg::LaserScan range;
    range.header.stamp = getStampMsgFromSec(io->currentTime());
    range.header.frame_id = sensor->name();
    range.range_max = sensor->maxDistance();
    range.range_min = sensor->minDistance();
    if (sensor->yawRange() == 0.0) {
        range.angle_max = sensor->pitchRange() / 2.0;
        range.angle_min = -sensor->pitchRange() / 2.0;
        range.angle_increment = sensor->pitchStep();
    } else {
        range.angle_max = sensor->yawRange() / 2.0;
        range.angle_min = -sensor->yawRange() / 2.0;
        range.angle_increment = sensor->yawStep();
    }
    range.ranges.resize(sensor->rangeData().size());
    //range.intensities.resize(sensor->rangeData().size());
    // for (size_t j = 0; j < sensor->rangeData().size(); ++j) {
    for (size_t j = 0; j < sensor->numYawSamples(); ++j) {
        range.ranges[j] = sensor->rangeData()[j];
        //range.intensities[j] = -900000;
    }
    publisher->publish(range);
}


void BodyROS2Item::update3DRangeSensor(
    RangeSensor *sensor,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher)
{
    if (!sensor->on()) {
        return;
    }
    sensor_msgs::msg::PointCloud range;
    // Header Info
    range.header.stamp = getStampMsgFromSec(io->currentTime());
    range.header.frame_id = sensor->name();

    // Calculate Point Cloud data
    const int numPitchSamples = sensor->numPitchSamples();
    const double pitchStep = sensor->pitchStep();
    const int numYawSamples = sensor->numYawSamples();
    const double yawStep = sensor->yawStep();

    for (int pitch = 0; pitch < numPitchSamples; ++pitch) {
        const double pitchAngle = pitch * pitchStep
                                  - sensor->pitchRange() / 2.0;
        const double cosPitchAngle = cos(pitchAngle);
        const int srctop = pitch * numYawSamples;

        for (int yaw = 0; yaw < numYawSamples; ++yaw) {
            const double distance = sensor->rangeData()[srctop + yaw];
            if (distance <= sensor->maxDistance()) {
                double yawAngle = yaw * yawStep - sensor->yawRange() / 2.0;
                geometry_msgs::msg::Point32 point;
                point.x = distance * cosPitchAngle * sin(-yawAngle);
                point.y = distance * sin(pitchAngle);
                point.z = -distance * cosPitchAngle * cos(-yawAngle);
                range.points.push_back(point);
            }
        }
    }

    publisher->publish(range);
}


void BodyROS2Item::input() {}

void BodyROS2Item::output() {}

void BodyROS2Item::switchDevice(
    std_srvs::srv::SetBool::Request::ConstSharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response,
    Device *sensor)
{
    sensor->on(request->data);
    response->success = (request->data == sensor->on());
}

void BodyROS2Item::stop()
{
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return;
}

builtin_interfaces::msg::Time BodyROS2Item::getStampMsgFromSec(double sec)
{
    builtin_interfaces::msg::Time msg;
    msg.sec = int(sec);
    msg.nanosec = (sec - int(sec)) * 1000000000;
    return msg;
}
