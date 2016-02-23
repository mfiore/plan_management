#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <plan_management_msgs/ManageSequentialPlanAction.h>
#include "plan_manager/sequential_plan_manager.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "sequential_plan_manager");

  ros::NodeHandle node_handle;

  string robot_name;
  vector<string> humans;

  node_handle.getParam("/robot/name",robot_name);
  node_handle.getParam("/human_agents/agent_names",humans);

  ROS_INFO("Init sequential_plan_manager");
  ROS_INFO("Robot name is %s",robot_name.c_str());
  ROS_INFO("Found %ld human agents with names:",humans.size());
  for (int i=0; i<humans.size();i++) {
	 ROS_INFO("- %s",humans[i].c_str());
  }

  SequentialPlanManager sequential_plan_manager(robot_name,humans);
  ros::spin();


  return 0;
}
