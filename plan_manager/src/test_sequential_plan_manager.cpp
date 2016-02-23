/*
 * test_sequential_plan_manager.cpp
 *
 *  Created on: Jul 21, 2015
 *      Author: mfiore
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "plan_management_msgs/ManageSequentialPlanAction.h"



typedef actionlib::SimpleActionClient<plan_management_msgs::ManageSequentialPlanAction> Client;

int main(int argc, char** argv) {
	ros::init(argc,argv,"test_sequential_plan_manager");

	ROS_INFO("Init test_sequential_plan_manager");
	Client action_client("/plan_management/manage_sequential_plan",true);
	action_client.waitForServer();
	ROS_INFO("Connected to manage sequential plan server");

	plan_management_msgs::ManageSequentialPlanGoal goal;

	goal.task_plan.plan_id=1;
	goal.task_plan.actors.push_back("PR2_ROBOT");

	plan_management_msgs::SequentialPlanActionList actionList;
	actionList.main_actor="PR2_ROBOT";
	plan_management_msgs::SequentialPlanAction plan_action;

	plan_action.main_actor="PR2_ROBOT";
	plan_action.action_links.push_back(1);

	action_management_msgs::Action action;
	action.id=0;
	action.name="take";
	common_msgs::Parameter p;
	p.value="GREY_TAPE";
	action.parameters.push_back(p);


	plan_action.action=action;
	actionList.actions.push_back(plan_action);
	goal.task_plan.actions.push_back(actionList);

	ROS_INFO("Sending goal");
	action_client.sendGoal(goal);
	ROS_INFO("Goal sent");

	ros::waitForShutdown();

}


