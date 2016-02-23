/**
author Michelangelo Fiore

class to represent a generic plan manager. 

**/

#ifndef SEQUENTIALPLANMANAGER_H
#define SEQUENTIALPLANMANAGER_H

#include "plan_management_msgs/ManageSequentialPlanAction.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>

#include <boost/foreach.hpp>


#include <boost/thread.hpp>


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include "plan_manager.h"

#include "ros/ros.h"



typedef actionlib::SimpleActionServer<plan_management_msgs::ManageSequentialPlanAction> Server;
typedef actionlib::SimpleActionClient<plan_management_msgs::ManageSequentialPlanAction> Client;
#include <boost/foreach.hpp>
using namespace std;

class SequentialPlanManager: public  PlanManager {
public:
	SequentialPlanManager(string robot_name, vector<string> human_agents);
protected:
	plan_management_msgs::ManageSequentialPlanFeedback feedback_;
	plan_management_msgs::ManageSequentialPlanResult result_;

private:
	void execute(const plan_management_msgs::ManageSequentialPlanGoalConstPtr& goal);
	void planDoneCb(const actionlib::SimpleClientGoalState& state,
            const plan_management_msgs::ManageSequentialPlanResultConstPtr& result);
	void planFeedbackCb(const plan_management_msgs::ManageSequentialPlanFeedbackConstPtr& feedback);	

	Server action_server_;
	Client manage_robot_client_;
	Client manage_human_client_;

	boost::condition_variable  plan_result_condition_;
	boost::mutex plan_result_mutex_;

	int plans_done_;
	int plans_failed_;

	map<string,plan_management_msgs::PlanReport> plan_reports_;
};


#endif
