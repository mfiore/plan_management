/**
Author Michelangelo Fiore

manages human actions, monitoring them one after the other.

*/

#include "plan_management_msgs/ManageSequentialPlanAction.h"
#include "situation_assessment_msgs/QueryDatabase.h"
#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/DatabaseRequest.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp> 

#include <actionlib/server/simple_action_server.h>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>

using namespace std;


boost::mutex mutex_should_provide_feedback_;
bool should_provide_feedback_;

boost::mutex mutex_plan_failed_;
bool plan_failed_;

boost::mutex mutex_plan_completed_;
int n_plans_completed_;

boost::mutex mutex_should_stop_;
bool should_stop_;

boost::mutex mutex_last_actions_;
vector<action_management_msgs::Action> last_actions_;

vector<string> actors_;


double database_condition_wait_rate_;
string robot_name_;

typedef actionlib::SimpleActionServer<plan_management_msgs::ManageSequentialPlanAction> Server; //action server type

ros::ServiceClient database_client_;
ros::ServiceClient add_fact_client_;


void setLastAction(int i, action_management_msgs::Action action) {
    boost::lock_guard<boost::mutex> lock(mutex_last_actions_);
    last_actions_[i]=action;
}

bool shouldProvideFeedback() {
    boost::lock_guard<boost::mutex> lock(mutex_should_provide_feedback_);
    return should_provide_feedback_;
}

void setShouldProvideFeedback(bool value) {
    boost::lock_guard<boost::mutex> lock(mutex_should_provide_feedback_);
    should_provide_feedback_=value;
}

void provideFeedback(Server *action_server) {
    plan_management_msgs::ManageSequentialPlanFeedback feedback;
    ros::Rate r(1);
    setShouldProvideFeedback(true);
    while (shouldProvideFeedback()) {
        feedback.plan_report.last_actions=last_actions_;
        feedback.plan_report.actors=actors_;
        feedback.plan_report.report.status="RUNNING";
        action_server->publishFeedback(feedback);
        r.sleep();
    } 
}

bool isPlanCompleted(int n_agents) {
    boost::lock_guard<boost::mutex> lock(mutex_plan_completed_);
    return n_plans_completed_==n_agents;
}

void setHumanFinished() {
    boost::lock_guard<boost::mutex> lock(mutex_plan_completed_);
    n_plans_completed_++;
}

void stopHumanPlans() {
    boost::lock_guard<boost::mutex> lock(mutex_should_stop_);
    should_stop_=true;
}

bool shouldStop() {
    boost::lock_guard<boost::mutex> lock(mutex_should_stop_);
    return should_stop_;
}

bool isPlanFailed() {
    boost::lock_guard<boost::mutex> lock(mutex_plan_failed_);
    return plan_failed_;
}
void setPlanFailed() {
    boost::lock_guard<boost::mutex> lock(mutex_plan_failed_);
    plan_failed_=true;
}

bool waitForLinks(vector<int> link_list) {
    ros::Rate r(database_condition_wait_rate_);

    ROS_INFO("SIMPLE_HUMAN_MANAGER Waiting for action links");
    BOOST_FOREACH(int link,link_list) {
        ROS_INFO("SIMPLE_HUMAN_MANAGER Waiting for action with id %d",link);
        situation_assessment_msgs::QueryDatabase srv;
        situation_assessment_msgs::Fact query_fact;
        
        query_fact.model=robot_name_;
        query_fact.subject="action_"+boost::lexical_cast<string>(link);
        query_fact.predicate.push_back("isDone");
        query_fact.value.push_back("true");

        srv.request.query=query_fact;

        bool is_link_done=false;

        //for each link we keep polling the DB until its realized or we are stopped.
        while(!is_link_done && !shouldStop()){
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
            ROS_INFO("SIMPLE_HUMAN_MANAGER Link %d completed",link);
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
        ROS_ERROR("SIMPLE_HUMAN_MANAGER failed to contact db");
    }
}


void monitorPlan(plan_management_msgs::SequentialPlanActionList plan, int agent_index) {
    //for each action wait for the links and then execute it
    for (int i=0; i<plan.actions.size();i++) {
        plan_management_msgs::SequentialPlanAction action_msg=plan.actions[i];
        if (shouldStop()) return;
        action_management_msgs::Action action=action_msg.action;
        vector<int> link_list=action_msg.action_links;

        ROS_INFO("SIMPLE_HUMAN_MANAGER Current action %s with id %d and parameters",action.name.c_str(), action.id);

        for(int i=0; i<action.parameters.size();i++) {
            ROS_INFO("SIMPLE_HUMAN_MANAGER %s",action.parameters[i].name.c_str());
            ROS_INFO("SIMPLE_HUMAN_MANAGER %s",action.parameters[i].value.c_str());
        }
        bool link_completed=waitForLinks(link_list);
        if (!link_completed) {
            if (shouldStop()) return;
            else {
                setPlanFailed();
                return;
            }
        }

        //execute the action
        ROS_INFO("SIMPLE_HUMAN_MANAGER Monitoring the action");
        setLastAction(agent_index,action);
        updateDatabase(boost::lexical_cast<string>(action.id));
    }
    ROS_INFO("SIMPLE_HUMAN_MANAGER actor %s has finished it's plan",plan.main_actor.c_str());
    setHumanFinished();
}



void execute(const plan_management_msgs::ManageSequentialPlanGoalConstPtr& goal, Server* action_server) {
        ROS_INFO("SIMPLE_HUMAN_MANAGER Received goal");

        plan_management_msgs::PlanReport human_report;
        action_management_msgs::Action last_action;
        plan_management_msgs::ManageSequentialPlanResult result;
        plan_management_msgs::ManageSequentialPlanFeedback feedback;

     

        int n_agents=goal->task_plan.actors.size();
        actors_=goal->task_plan.actors;

        human_report.plan_id=goal->task_plan.plan_id;
        human_report.actors=actors_;
        ROS_INFO("Human report actors number %ld",actors_.size());

        should_provide_feedback_=false;
        plan_failed_=false;
        n_plans_completed_=0;
        should_stop_=0;
        last_actions_=vector<action_management_msgs::Action>(n_agents);

        if (goal->task_plan.actions.size()==0) {
            ROS_ERROR("SIMPLE_HUMAN_MANAGER no actions in goal");
            human_report.report.status="FAILED";
            human_report.report.details="no actions in goal";
            action_server->setAborted(result);
        }
        vector<plan_management_msgs::SequentialPlanAction> action_list=goal->task_plan.actions[0].actions;

        for (int i=0; i<n_agents;i++) {
            if (action_list[i].main_actor==robot_name_) {
                ROS_WARN("Wrong actor. This node only accepts actions for human agents");
                human_report.report.status="FAILED";
                human_report.report.details="wrong actor";
                result.plan_report=human_report;
                action_server->setAborted(result);
                return;
            }
        }

        vector<boost::thread*> human_threads;
        for (int i=0; i<goal->task_plan.actors.size();i++) {
            boost::thread t(&monitorPlan,goal->task_plan.actions[i],i);
            human_threads.push_back(&t);
        }
        setShouldProvideFeedback(true);
        ros::Rate r(1);
        while (!isPlanCompleted(n_agents) && ros::ok() && !isPlanFailed() && !action_server->isPreemptRequested()) {
            r.sleep();
        }
        setShouldProvideFeedback(false);
        if (!isPlanCompleted(n_agents)) {
            stopHumanPlans();
        }
        for (int i=0;i<human_threads.size();i++) {
            human_threads[i]->join();
        }
        if (!ros::ok()) return;

        if (isPlanCompleted(n_agents)) {
            human_report.report.status="COMPLETED";
            human_report.last_actions.push_back(last_action);
            ROS_INFO("SIMPLE_HUMAN_MANAGER Action sequence completed");
            result.plan_report=human_report;
            action_server->setSucceeded(result);
        }
        else if (action_server->isPreemptRequested()) {
            human_report.report.status="PREEMPTED";
            human_report.last_actions.push_back(last_action);
            ROS_INFO("SIMPLE_HUMAN_MANAGER preempted");
            result.plan_report=human_report;
            action_server->setPreempted(result);
        }
        else {
            human_report.report.status="FAILED";
            human_report.report.details="";
            human_report.last_actions.push_back(last_action);
            ROS_INFO("SIMPLE_HUMAN_MANAGER monitor failed");
            result.plan_report=human_report;
            action_server->setPreempted(result);            
        }



}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "SIMPLE_HUMAN_MANAGER");
    ros::NodeHandle node_handle;

    node_handle.getParam("/robot/name",robot_name_);
    node_handle.getParam("/database_condition_wait_rate",database_condition_wait_rate_);

    ROS_INFO("SIMPLE_HUMAN_MANAGER Init simple_human_manager");
    ROS_INFO("SIMPLE_HUMAN_MANAGER Robot name is %s",robot_name_.c_str());
    ROS_INFO("SIMPLE_HUMAN_MANAGER Condition wait rate is %f",database_condition_wait_rate_);

    database_client_ = node_handle.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
    database_client_.waitForExistence(); 

    add_fact_client_ = node_handle.serviceClient<situation_assessment_msgs::DatabaseRequest>("situation_assessment/add_facts");
    add_fact_client_.waitForExistence();

    ROS_INFO("Connected to database");


    Server action_server(node_handle, "/plan_management/manage_sequential_human_actions", 
        boost::bind(&execute, _1, &action_server), false);
    ROS_INFO("Starting action server to manage human actions: name: simple_human_manager , type:sequential");
    action_server.start();

    ros::spin();
    return 0;
}
