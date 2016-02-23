/**
author Michelangelo Fiore

abstract class to represent a generic plan manager. 

**/

#include "plan_manager/plan_manager.h"

PlanManager::PlanManager(string robot_name, vector<string> human_agents):
robot_name_(robot_name),
human_agents_(human_agents){

}
