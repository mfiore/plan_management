#include "plan_manager/sequential_plan_manager.h"

SequentialPlanManager::SequentialPlanManager(string robot_name, vector<string> human_agents):
PlanManager(robot_name,human_agents),
action_server_(node_handle_, "plan_management/manage_sequential_plan", 
	boost::bind(&SequentialPlanManager::execute,this, _1), false),
 manage_robot_client_("plan_management/manage_sequential_robot_actions",true),
 manage_human_client_("plan_management/manage_sequential_human_actions",true)
 {
 	plans_done_=0;
 	plans_failed_=0;



  	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Waiting for agent manager servers");
 	manage_robot_client_.waitForServer();
 	// manage_human_client.waitForServer();
 	action_server_.start();
 	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Started server manage_sequential_plan");
 	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Ready");
}

void SequentialPlanManager::planDoneCb(const actionlib::SimpleClientGoalState& state,
        const plan_management_msgs::ManageSequentialPlanResultConstPtr& result) {
	boost::lock_guard<boost::mutex> lock(plan_result_mutex_);
	if (result->plan_report.report.status=="COMPLETED") {
		plans_done_++;
	}
	else {
		plans_failed_++;
	}

	plan_reports_[result->plan_report.actors[0]]=result->plan_report;

	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Actor %s completed its plan with result %s",result->plan_report.actors[0].c_str(),result->plan_report.report.status.c_str());

	plan_result_condition_.notify_one();
}

//if we receive feedback we just publish it 
void SequentialPlanManager::planFeedbackCb(const plan_management_msgs::ManageSequentialPlanFeedbackConstPtr& feedback){
	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Received feedback:");
	for (int i=0; i<feedback->plan_report.actors.size();i++) {
		action_management_msgs::Action feedback_action=feedback->plan_report.last_actions[i];
		string action_string=feedback_action.name;
		for (int j=0; j<feedback_action.parameters.size();j++) {
			action_string=action_string+" "+feedback_action.parameters[j].name+" -"+feedback_action.parameters[j].value;
		}
		ROS_INFO("SEQUENTIAL_PLAN_MANAGER %s completed %s",feedback->plan_report.actors[i].c_str(),action_string.c_str());
	}
	action_server_.publishFeedback(feedback);
}	


void SequentialPlanManager::execute(const plan_management_msgs::ManageSequentialPlanGoalConstPtr& goal) {
	plan_management_msgs::SequentialTaskPlan robot_plan;
	plan_management_msgs::SequentialTaskPlan human_plan;

	plans_done_=0;
	plans_failed_=0;


	plan_management_msgs::SequentialTaskPlan plan=goal->task_plan;


	plan_management_msgs::PlanReport total_report;
	total_report.plan_id=plan.plan_id;
	total_report.actors=plan.actors;

	int received_plans=0;

	robot_plan.plan_id=plan.plan_id;
	human_plan.plan_id=plan.plan_id;

	ROS_INFO("SEQUENTIAL_PLAN_MANAGER Received goal with id %d",plan.plan_id);


	vector<plan_management_msgs::SequentialPlanActionList> plan_actions=plan.actions;
	vector<string> human_actors;
	//each plan contains a list of actions related to an agent. We scan this list and send
	//the action to the correct manager, which is an action_lib server that uses the same
	//ManageSequentialPlan action type.
	BOOST_FOREACH(plan_management_msgs::SequentialPlanActionList agent_actions,plan_actions)
	{
		if (agent_actions.main_actor==robot_name_) {

			received_plans++;

			robot_plan.actors.push_back(robot_name_);

			robot_plan.actions.push_back(agent_actions);
			plan_management_msgs::ManageSequentialPlanGoal robot_goal;
			robot_goal.task_plan=robot_plan;


			ROS_INFO("SEQUENTIAL_PLAN_MANAGER Found a plan for %s . Sending goal",robot_name_.c_str());

			manage_robot_client_.sendGoal(robot_goal,
					boost::bind(&SequentialPlanManager::planDoneCb,this, _1,_2),
				 	Client::SimpleActiveCallback(),
				 	boost::bind(&SequentialPlanManager::planFeedbackCb,this, _1)
			);
		}
		else if (std::find(human_agents_.begin(),human_agents_.end(),agent_actions.main_actor)!=human_agents_.end()){
			human_plan.actions.push_back(agent_actions);
			human_plan.actors.push_back(agent_actions.main_actor);
		}
	}
	if (human_plan.actions.size()!=0) {
		received_plans++;

		plan_management_msgs::ManageSequentialPlanGoal human_goal;
		human_goal.task_plan=human_plan;

		ROS_INFO("SEQUENTIAL_PLAN_MANAGER Found plan for %ld human agents. Sending goal.",human_plan.actions.size());

		manage_human_client_.sendGoal(human_goal,
				boost::bind(&SequentialPlanManager::planDoneCb,this,_1,_2),
			 	Client::SimpleActiveCallback(),
			 	boost::bind(&SequentialPlanManager::planFeedbackCb,this,_1)
			 	);
	}
	//start waiting for the agents to finish their plans
	boost::unique_lock<boost::mutex> lock(plan_result_mutex_);
	while(plans_done_!=received_plans && plans_failed_==0  && !(action_server_.isPreemptRequested()) && ros::ok()) {
		plan_result_condition_.wait(lock);
	}

	if (action_server_.isPreemptRequested() || !ros::ok()) {
		ROS_INFO("SEQUENTIAL_PLAN_MANAGER Goal is preempted. Stopping management");

		std::pair<string,plan_management_msgs::PlanReport> report;
		BOOST_FOREACH(report, plan_reports_) {
			total_report.last_actions.insert(total_report.last_actions.end(),report.second.last_actions.begin(),report.second.last_actions.end());
		}
		manage_robot_client_.cancelGoal();
		manage_human_client_.cancelGoal();

		total_report.report.status="PREEMPTED";

		result_.plan_report=total_report;
		action_server_.setPreempted(result_);
		return;
	}

	if (received_plans==0) {
		ROS_WARN("SEQUENTIAL_PLAN_MANAGER No valid plans found in the goal. Aborting");
		total_report.report.status="FAILED";
		total_report.report.details="No valid plans received";
	}

	//create final report by joining the human and robot reports.
	else if (plans_failed_>0) {
		ROS_INFO("SEQUENTIAL_PLAN_MANAGER Goal is failed");
		total_report.report.status="FAILED";
		total_report.report.details=plans_failed_+" agents failed their goals";
	}
	else
	{
		ROS_INFO("SEQUENTIAL_PLAN_MANAGER Goal completed");
		total_report.report.status="COMPLETED";
	}

	std::pair<string,plan_management_msgs::PlanReport> report;
	BOOST_FOREACH(report, plan_reports_) {
		total_report.last_actions.insert(total_report.last_actions.end(),report.second.last_actions.begin(),report.second.last_actions.end());
	}

	result_.plan_report=total_report;

	if (total_report.report.status=="COMPLETED") {
		action_server_.setSucceeded(result_);
	}
	else {
		action_server_.setAborted(result_);
	}

}

