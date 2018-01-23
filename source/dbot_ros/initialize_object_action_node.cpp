#include <string>

#include "actionlib/server/simple_action_server.h"
#include "dbot_ros/util/interactive_marker_initializer.h"
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

private:
    std::string action_name_;
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dbot_ros_msgs::InitializeObjectAction> as_;
};

InitializeObjectAction::InitializeObjectAction(const std::string& action_name)
    : action_name_(action_name),
      nh_(),
      as_(nh_,
          action_name,
          boost::bind(&InitializeObjectAction::Execute, this, _1),
          false)
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
    ROS_INFO(
        "Please use rviz to align and initialize the object poses under the "
        "topic.");
    ROS_INFO("Waiting for all interactive object poses to be set ...");
    while (ros::ok() && !object_init.are_all_object_poses_set())
    {
        if (as_.isPreemptRequested())
        {
            as_.setPreempted();
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
}  // namespace pbi

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initialize_object_action_server");
    ros::NodeHandle nh;
    pbi::InitializeObjectAction action("initialize_object");
    action.Start();
    ros::spin();
    return 0;
}
