#include "dbot_ros/object_tracker.h"

#include <memory>
#include <string>

#include "dbot/object_resource_identifier.h"
#include "dbot_ros/object_tracker_ros.h"
#include "dbot_ros/util/ros_interface.h"
#include "dbot_ros_msgs/ObjectState.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "dbot_ros/util/particle_tracker_builder.h"

namespace pbi
{
ObjectTracker::ObjectTracker()
    : name_(""), mesh_name_(""), nh_(), object_tracker_()
{
}

void ObjectTracker::Instantiate(const std::string& name,
                                const std::string& mesh_name,
                                const sensor_msgs::CameraInfo& camera_info)
{
    name_      = name;
    mesh_name_ = mesh_name;

    dbot::ObjectResourceIdentifier ori;

    pbi::ParticleTrackerBuilder object_tracker_builder(nh_, camera_info);
    pbi::BuildOri(nh_, mesh_name, &ori);
    mesh_package_ = ori.package();
    mesh_dir_     = ori.directory();
    object_tracker_builder.set_object(ori);
    object_tracker_.reset();
    object_tracker_ = object_tracker_builder.BuildRos();
}

void ObjectTracker::SetPose(const geometry_msgs::Pose& pose)
{
    object_tracker_->tracker()->initialize({ri::to_pose_velocity_vector(pose)});
}

void ObjectTracker::Step(const sensor_msgs::Image& depth)
{
    if (!object_tracker_)
    {
        ROS_ERROR("Object tracker for \"%s\" not instantiated", name_.c_str());
        return;
    }
    if (depth.encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&
        depth.encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
        ROS_ERROR("Unsupported depth encoding %s", depth.encoding.c_str());
        return;
    }
    if (depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        object_tracker_->update_obsrv(depth);
    }
    else if (depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        // Encodes millimeters, convert to float
        sensor_msgs::Image depth_float;
        depth_float.header       = depth.header;
        depth_float.height       = depth.height;
        depth_float.width        = depth.width;
        depth_float.encoding     = sensor_msgs::image_encodings::TYPE_32FC1;
        depth_float.is_bigendian = false;
        depth_float.step         = sizeof(float) * depth.width;
        const uint16_t* data_16 =
            reinterpret_cast<const uint16_t*>(depth.data.data());
        float data_float[depth.height * depth.width];
        for (unsigned int row = 0; row < depth.height; ++row)
        {
            for (unsigned int col = 0; col < depth.width; ++col)
            {
                int index         = row * depth.width + col;
                data_float[index] = data_16[index] / 1000.0;
            }
        }
        uint8_t* data_8 = reinterpret_cast<uint8_t*>(data_float);
        depth_float.data.assign(
            data_8, data_8 + sizeof(float) * depth.height * depth.width);

        object_tracker_->update_obsrv(depth_float);
    }
    object_tracker_->run_once();
}

void ObjectTracker::GetPose(geometry_msgs::PoseStamped* pose_stamped) const
{
    if (!object_tracker_)
    {
        ROS_ERROR("Object tracker for \"%s\" not instantiated", name_.c_str());
        return;
    }
    *pose_stamped = object_tracker_->current_pose();
}

void ObjectTracker::GetPose(geometry_msgs::Pose* pose) const
{
    if (!object_tracker_)
    {
        ROS_ERROR("Object tracker for \"%s\" not instantiated", name_.c_str());
        return;
    }
    *pose = object_tracker_->current_pose().pose;
}

std::string ObjectTracker::name() const
{
    return name_;
}
std::string ObjectTracker::mesh_name() const
{
    return mesh_name_;
}
}  // namespace pbi
