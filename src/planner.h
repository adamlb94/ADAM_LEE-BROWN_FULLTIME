#ifndef PLANNER_H
#define PLANNER_H

#include "common.h"
#include "agent_plan.h"
#include "multi_agent_planning/AgentPos.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/Position.h"
#include "path_cache.h"
#include "roadmap.h"
#include "ros/ros.h"

#include <queue>
#include <unordered_map>

/* Roadmap width */
#define WIDTH 11

/* Roadmap height */
#define HEIGHT 11

/**
 * Planning node that plans paths for agents.
 */
class Planner {
private:
    struct CellOccupation {
        /* Seconds from t0 at which an agent will be centered at a point */
        int time;
        /* Angle of the robot's movement from a point in degrees */
        int movementAngle;

        CellOccupation(int time, int movementAngle);
    };

    /* ROS node handle */
    std::unique_ptr<ros::NodeHandle> nodeHandle;

    /* Subscriber for AGENT_FEEDBACK_TOPIC */
    ros::Subscriber agentFeedbackSubscriber;

    /* Server for GET_PLAN_SERVICE */
    ros::ServiceServer getPlanServer;

    /* All agents known to the planner */
    std::unordered_map<std::string, AgentPlan> agents;

    /* The roadmap (grid) */
    Roadmap roadmap;

    /* Cache of previously-calculated paths */
    PathCache pathCache;

    void agentFeedbackCallback(const multi_agent_planning::AgentPos::ConstPtr& msg);
    multi_agent_planning::Position position(int x, int y);
    CellOccupation calculateCellOccupation(std::vector<std::vector<CellOccupation>> occupations, int startX, int startY);

    bool isOccupied(int x, int y, CellOccupation occupation);
    bool exploreCoord(multi_agent_planning::Position pos, multi_agent_planning::Position endPos, std::vector<std::vector<CellOccupation>> *bfsFromStartOccupations, std::vector<std::vector<CellOccupation>> *bfsFromEndOccupations, std::vector<std::vector<multi_agent_planning::Position>> *prevPos, std::queue<multi_agent_planning::Position> *queue, multi_agent_planning::Position *intersectingPos);
    bool planPath(multi_agent_planning::Position startPos, multi_agent_planning::Position goalPos, std::vector<std::vector<multi_agent_planning::Position>> *prevPos, std::vector<std::vector<multi_agent_planning::Position>> *nextPos, multi_agent_planning::Position *intersectingPos);
    void constructPath(std::vector<multi_agent_planning::Position> *path, multi_agent_planning::Position startPos, multi_agent_planning::Position goalPos, multi_agent_planning::Position intersectingPos, std::vector<std::vector<multi_agent_planning::Position>> *prevPos, std::vector<std::vector<multi_agent_planning::Position>> *nextPos);
    std::vector<multi_agent_planning::Position> getShortestPath(multi_agent_planning::Position startPos, multi_agent_planning::Position goalPos);
    bool getPlanCallback(multi_agent_planning::GetPlan::Request &req, multi_agent_planning::GetPlan::Response &res);

public:
    void init(int argc, char **argv);
    void execute();
};

#endif
