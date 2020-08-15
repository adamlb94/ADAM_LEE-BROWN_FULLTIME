#ifndef AGENT_H
#define AGENT_H

#include "common.h"
#include "multi_agent_planning/Position.h"
#include "multi_agent_planning/UpdateGoal.h"
#include "ros/ros.h"

/**
 * Agent node that requests paths from the planner.
 */
class Agent {
private:
    /* Serial ID */
    std::string id;

    /* Current position */
    multi_agent_planning::Position pos;

    /* Goal position */
    multi_agent_planning::Position goalPos;

    /* Planned path including the current and goal position */
    std::vector<multi_agent_planning::Position> path;

    /* ROS node handle */
    std::unique_ptr<ros::NodeHandle> nodeHandle;

    /* Publisher for AGENT_FEEDBACK_TOPIC */
    ros::Publisher agentFeedbackPublisher;

    /* Server for UPDATE_GOAL_SERVICE */
    ros::ServiceServer updateGoalServer;

    /* Client for GET_PLAN_SERVICE */
    ros::ServiceClient getPlanClient;

    /* Indicates that an end goal has been received from UPDATE_GOAL_SERVICE */
    bool endGoalReceived;

    int publishPos();
    bool updateGoalCallback(multi_agent_planning::UpdateGoal::Request &req, multi_agent_planning::UpdateGoal::Response &res);
    void getPlan();

public:
    int init(int argc, char **argv);
    int execute();
};

#endif
