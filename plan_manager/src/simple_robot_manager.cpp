/**
Author Michelangelo Fiore

manages robot actions, simply executing them one by one after waiting for preconditions.

*/

#include "plan_management_msgs/ManageSequentialPlanAction.h"
#include "situation_assessment_msgs/QueryDatabase.h"
#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/DatabaseRequest.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

using namespace std;

typedef actionlib::SimpleActionServer<plan_management_msgs::ManageSequentialPlanAction> Server; //action server type
ros::ServiceClient database_client_;  //used to query for action links 
ros::ServiceClient add_fact_client_;  //used to query for action links 

string robot_name_;  
double database_condition_wait_rate_;  //how much to wait between queries to the db

/*
utility function to send a failed report and abandon the current goal
*/
void fail(Server *action_server, action_management_msgs::Action last_action) {
	plan_management_msgs::ManageSequentialPlanResult result;
	plan_management_msgs::PlanReport robot_report;

	ROS_INFO("SIMPLE_ROBOT_MANAGER Plan failed");


	robot_report.report.status="FAILED";
	robot_report.actors.push_back(robot_name_);
	robot_report.last_actions.push_back(last_action);
	result.plan_report=robot_report;
	action_server->setAborted(result);

}

/*
utility function to check if the goal is  preempted, and in that case send a preempted report.
*/
bool isPreempted(Server *action_server, action_management_msgs::Action last_action) {
	plan_management_msgs::ManageSequentialPlanResult result;
	plan_management_msgs::PlanReport robot_report;

	if (action_server->isPreemptRequested()) {
		ROS_INFO("SIMPLE_ROBOT_MANAGER Goal is preempted");
		robot_report.report.status="PREEMPTED";
		robot_report.report.details="goal canceled";
		robot_report.last_actions.push_back(last_action);
		robot_report.actors.push_back(robot_name_);
		result.plan_report=robot_report;
		action_server->setPreempted(result);

		return true;
	}
	else {
		return false;
	}
}

/*
wait for the previous actions in the plan to be completed. Returns true if they are completed successfully,
false otherwise.
*/
bool waitForLinks(vector<int> link_list, Server *action_server) {
	ros::Rate r(database_condition_wait_rate_);

	ROS_INFO("SIMPLE_ROBOT_MANAGER Waiting for action links");
	BOOST_FOREACH(int link,link_list) {
		ROS_INFO("SIMPLE_ROBOT_MANAGER Waiting for action with id %d",link);
		situation_assessment_msgs::QueryDatabase srv;
		situation_assessment_msgs::Fact query_fact;
		
		query_fact.model=robot_name_;
		query_fact.subject="action_"+boost::lexical_cast<string>(link);
		query_fact.predicate.push_back("isDone");
		query_fact.value.push_back("true");

		srv.request.query=query_fact;

		bool is_link_done=false;

		//for each link we keep polling the DB until its realized or we are stopped.
		while(!is_link_done && ros::ok() && !action_server->isPreemptRequested()){
			if (database_client_.call(srv)) {
				is_link_done=srv.response.result.size()>0;
			}
			else {
				ROS_ERROR("Error: failed to contact query_database");
				return false; 
			}
			if (!is_link_done) {
				r.sleep();
			}
		}
		if (!is_link_done){
			return false;  
		}
		else {
			ROS_INFO("SIMPLE_ROBOT_MANAGER Link %d completed",link);
		}
	}
	return true;
}

void updateDatabase(string action_id) {
	situation_assessment_msgs::DatabaseRequest srv;
	situation_assessment_msgs::Fact fact;
	fact.model=robot_name_;
	fact.subject="action_"+action_id;
	fact.predicate.push_back("isDone");
	fact.value.push_back("true");

	srv.request.fact_list.push_back(fact);

	if (!add_fact_client_.call(srv)) {
		ROS_ERROR("SIMPLE_ROBOT_MANAGER failed to contact db");
	}
}

/*
main procedure to manage the robot's actions
*/
 void execute(const plan_management_msgs::ManageSequentialPlanGoalConstPtr& goal, Server* action_server) {
 	ROS_INFO("SIMPLE_ROBOT_MANAGER Received goal");

 	plan_management_msgs::PlanReport robot_report;
 	action_management_msgs::Action last_action;
 	plan_management_msgs::ManageSequentialPlanResult result;
 	plan_management_msgs::ManageSequentialPlanFeedback feedback;

 	robot_report.plan_id=goal->task_plan.plan_id;
 	robot_report.actors.push_back(robot_name_);


 	plan_management_msgs::SequentialPlanActionList action_list=goal->task_plan.actions[0];

 	if (goal->task_plan.actors.size()>1) {
 		ROS_WARN("Too many actors in request. This node only accepts actions for %s",robot_name_.c_str());
		robot_report.report.status="FAILED";
		robot_report.report.details="wrong actor";
		result.plan_report=robot_report;
		action_server->setAborted(result);
		return;
 	}
 	if (action_list.main_actor!=robot_name_) { //check if there is an error in the received goal.
 		ROS_WARN("Wrong Actor %s. This node only accepts actions for %s",action_list.main_actor.c_str(),robot_name_.c_str());
 		robot_report.report.status="FAILED";
 		robot_report.report.details="wrong actor";
 		result.plan_report=robot_report;
 		action_server->setAborted(result);
 		return;
 	}

 	//for each action wait for the links and then execute it
 	BOOST_FOREACH(plan_management_msgs::SequentialPlanAction action_msg,action_list.actions) {
 		action_management_msgs::Action action=action_msg.action;
 		vector<int> link_list=action_msg.action_links;

 		ROS_INFO("SIMPLE_ROBOT_MANAGER Current action %s with id %d and parameters",action.name.c_str(), action.id);
		for(int i=0; i<action.parameters.size();i++) {
 			ROS_INFO("SIMPLE_ROBOT_MANAGER %s",action.parameters[i].name.c_str());
 			ROS_INFO("SIMPLE_ROBOT_MANAGER %s",action.parameters[i].value.c_str());
 		}
 		bool link_completed=waitForLinks(link_list,action_server);
 		if (!link_completed) {
 			if (isPreempted(action_server,last_action)) return; 
 			else {
 				fail(action_server,last_action);
 				return;
 			}
 		}

 		//execute the action
 		ROS_INFO("SIMPLE_ROBOT_MANAGER Executing the action");


 		//provide feedback
 		last_action=action_msg.action;

 		robot_report.report.status="RUNNING";
 		robot_report.last_actions.push_back(action);

 		feedback.plan_report=robot_report;
 		action_server->publishFeedback(feedback);


 		updateDatabase(boost::lexical_cast<string>(action.id));

 		if (isPreempted(action_server,last_action)) return;
 	}
 	robot_report.report.status="COMPLETED";
 	robot_report.last_actions.push_back(last_action);

 	ROS_INFO("SIMPLE_ROBOT_MANAGER Action sequence completed");
 	result.plan_report=robot_report;
 	action_server->setSucceeded(result);

 }


int main(int argc, char ** argv) {
    ros::init(argc, argv, "simple_robot_manager");
    ros::NodeHandle node_handle;

    node_handle.getParam("/robot/name",robot_name_);
    node_handle.getParam("/database_condition_wait_rate",database_condition_wait_rate_);

    ROS_INFO("SIMPLE_ROBOT_MANAGER Init simple_robot_manager");
    ROS_INFO("SIMPLE_ROBOT_MANAGER Robot name is %s",robot_name_.c_str());
    ROS_INFO("SIMPLE_ROBOT_MANAGER Condition wait rate is %f",database_condition_wait_rate_);

    database_client_ = node_handle.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
    database_client_.waitForExistence();   

    add_fact_client_ = node_handle.serviceClient<situation_assessment_msgs::DatabaseRequest>("situation_assessment/add_facts");
    add_fact_client_.waitForExistence();

    ROS_INFO("SIMPLE_ROBOT_MANAGER Connected to database");

    Server action_server(node_handle, "plan_management/manage_sequential_robot_actions", boost::bind(&execute, _1, &action_server), false);
    ROS_INFO("SIMPLE_ROBOT_MANAGER Starting action server to manage robot actions: name: simple_robot_manager , type:sequential");
    action_server.start();

    ros::spin();
    return 0;
}
