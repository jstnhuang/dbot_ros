#include <map>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "dbot_ros/util/interactive_marker_initializer.h"
#include "dbot_ros_msgs/InitInteractiveObjectPose.h"
#include "dbot_ros_msgs/InitializeObjectAction.h"
#include "ros/ros.h"

namespace pbi
{
class InitializeObjectAction
{
public:
    InitializeObjectAction(const std::string& action_name);
    void Start();
    void Execute(const dbot_ros_msgs::InitializeObjectGoalConstPtr& goal);
    bool HandleSetPose(dbot_ros_msgs::InitInteractiveObjectPoseRequest& req,
                       dbot_ros_msgs::InitInteractiveObjectPoseResponse& res);

private:
    std::string action_name_;
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dbot_ros_msgs::InitializeObjectAction> as_;
    std::map<std::string, geometry_msgs::Pose> poses_;
};

InitializeObjectAction::InitializeObjectAction(const std::string& action_name)
    : action_name_(action_name),
      nh_(),
      as_(nh_,
          action_name,
          boost::bind(&InitializeObjectAction::Execute, this, _1),
          false),
      poses_()
{
}

void InitializeObjectAction::Start()
{
    as_.start();
}

void InitializeObjectAction::Execute(
    const dbot_ros_msgs::InitializeObjectGoalConstPtr& goal)
{
    opi::InteractiveMarkerInitializer object_init(goal->frame_id);
    object_init.set_object("object_meshes",
                           "object_models",
                           goal->mesh_name,
                           goal->initial_pose,
                           false);
    ROS_INFO("Initializing %s", goal->mesh_name.c_str());
    ROS_INFO("Waiting for the object poses to be set in RViz");
    while (ros::ok() && !object_init.are_all_object_poses_set())
    {
        if (as_.isPreemptRequested())
        {
            as_.setPreempted();
            return;
        }
        if (poses_.find(goal->mesh_name) != poses_.end())
        {
            dbot_ros_msgs::InitializeObjectResult result;
            result.pose = poses_.at(goal->mesh_name);
            as_.setSucceeded(result);
            return;
        }
        ros::spinOnce();
    }
    if (object_init.poses().size() == 0)
    {
        ROS_ERROR("Unable to initialize object!");
        as_.setAborted();
        return;
    }

    dbot_ros_msgs::InitializeObjectResult result;
    result.pose = object_init.poses()[0];
    as_.setSucceeded(result);
}

bool InitializeObjectAction::HandleSetPose(
    dbot_ros_msgs::InitInteractiveObjectPoseRequest& req,
    dbot_ros_msgs::InitInteractiveObjectPoseResponse& res)
{
    poses_[req.mesh_name] = req.pose;
    return true;
}
}  // namespace pbi

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initialize_object_action_server");
    ros::NodeHandle nh;
    pbi::InitializeObjectAction action("initialize_object");
    ros::ServiceServer server =
        nh.advertiseService("init_interactive_object_pose",
                            &pbi::InitializeObjectAction::HandleSetPose,
                            &action);
    action.Start();
    ros::spin();
    return 0;
}
