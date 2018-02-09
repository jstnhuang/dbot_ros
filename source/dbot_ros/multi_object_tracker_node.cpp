#include <map>
#include <string>

#include "dbot_ros_msgs/MultiTrack.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "dbot_ros/object_tracker.h"

namespace msgs = dbot_ros_msgs;

namespace pbi
{
class MultiObjectTracker
{
public:
    MultiObjectTracker();
    bool HandleRequest(msgs::MultiTrackRequest& req,
                       msgs::MultiTrackResponse& res);
    void Create(const std::string& name,
                const std::string& mesh_name,
                const sensor_msgs::CameraInfo& camera_info);
    void Destroy(const std::string& name);
    void DestroyAll();
    void SetPose(const std::string& name, const geometry_msgs::Pose& pose);
    void Step(const std::string& name, const sensor_msgs::Image& depth_image);
    bool GetPose(const std::string& name, geometry_msgs::Pose* pose);

private:
    std::map<std::string, ObjectTracker> trackers_;
};

MultiObjectTracker::MultiObjectTracker() : trackers_()
{
}

bool MultiObjectTracker::HandleRequest(msgs::MultiTrackRequest& req,
                                       msgs::MultiTrackResponse& res)
{
    if (req.type == msgs::MultiTrackRequest::CREATE)
    {
        Create(req.object_name, req.mesh_name, req.camera_info);
    }
    else if (req.type == msgs::MultiTrackRequest::DESTROY)
    {
        Destroy(req.object_name);
    }
    else if (req.type == msgs::MultiTrackRequest::DESTROY_ALL)
    {
        DestroyAll();
    }
    else if (req.type == msgs::MultiTrackRequest::SET_POSE)
    {
        SetPose(req.object_name, req.pose);
    }
    else if (req.type == msgs::MultiTrackRequest::STEP)
    {
        Step(req.object_name, req.depth_image);
    }
    else if (req.type == msgs::MultiTrackRequest::GET_POSE)
    {
        return GetPose(req.object_name, &res.pose);
    }
    else
    {
        ROS_ERROR("Unsupported service type: \"%s\"", req.type.c_str());
        return false;
    }
    return true;
}

void MultiObjectTracker::Create(const std::string& name,
                                const std::string& mesh_name,
                                const sensor_msgs::CameraInfo& camera_info)
{
    if (trackers_.find(name) != trackers_.end())
    {
        ROS_ERROR("Tracker for \"%s\" already exists!", name.c_str());
        return;
    }
    trackers_[name].Instantiate(name, mesh_name, camera_info);
}

void MultiObjectTracker::Destroy(const std::string& name)
{
    if (trackers_.find(name) == trackers_.end())
    {
        ROS_ERROR("Tracker for \"%s\" not created yet!", name.c_str());
        return;
    }

    trackers_.erase(name);
}

void MultiObjectTracker::DestroyAll()
{
    trackers_.clear();
}

void MultiObjectTracker::SetPose(const std::string& name,
                                 const geometry_msgs::Pose& pose)
{
    if (trackers_.find(name) == trackers_.end())
    {
        ROS_ERROR("Tracker for \"%s\" not created yet!", name.c_str());
        return;
    }
    trackers_[name].SetPose(pose);
}

void MultiObjectTracker::Step(const std::string& name,
                              const sensor_msgs::Image& depth_image)
{
    if (trackers_.find(name) == trackers_.end())
    {
        ROS_ERROR("Tracker for \"%s\" not created yet!", name.c_str());
        return;
    }
    trackers_[name].Step(depth_image);
}

bool MultiObjectTracker::GetPose(const std::string& name,
                                 geometry_msgs::Pose* pose)
{
    if (trackers_.find(name) == trackers_.end())
    {
        ROS_ERROR("Tracker for \"%s\" not created yet!", name.c_str());
        return false;
    }
    trackers_[name].GetPose(pose);
    return true;
}
}  // namespace pbi

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dbot_multi_object_tracker");
    ros::NodeHandle nh;

    std::string object_name(argv[1]);

    pbi::MultiObjectTracker tracker;
    ros::ServiceServer server =
        nh.advertiseService("multi_object_track/" + object_name,
                            &pbi::MultiObjectTracker::HandleRequest,
                            &tracker);

    ros::spin();
    return 0;
}
