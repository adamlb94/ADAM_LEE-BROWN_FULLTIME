#ifndef COMMON_H
#define COMMON_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "multi_agent_planning/AgentPos.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/Position.h"
#include "multi_agent_planning/UpdateGoal.h"
#include <vector>

#define AGENT_FEEDBACK_TOPIC "agent_feedback"
#define GET_PLAN_SERVICE "get_plan"
#define UPDATE_GOAL_SERVICE "update_goal"
#define QUEUE_SIZE 1000

#endif