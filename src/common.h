#include "ros/ros.h"
#include "std_msgs/String.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/AgentPos.h"

static const std::string AGENT_FEEDBACK_TOPIC = "agent_feedback";
static const std::string GET_PLAN_SERVICE = "get_plan";
static const int QUEUE_SIZE = 1000;