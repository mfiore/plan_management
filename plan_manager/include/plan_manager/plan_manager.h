/**
author Michelangelo Fiore

abstract class to represent a generic plan manager. 

**/

#ifndef PLANMANAGER_H
#define PLANMANAGER_H

#include <string>
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace std;

class PlanManager {

public:
	PlanManager(string robot_name, vector<string> human_agents);
protected:
		ros::NodeHandle node_handle_;

		string robot_name_;
		vector<string> human_agents_;
private:

};


#endif