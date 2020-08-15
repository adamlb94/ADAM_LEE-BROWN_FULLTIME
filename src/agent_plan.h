#ifndef AGENT_H
#define AGENT_H

#include "multi_agent_planning/Position.h"

/**
 * Planning node's representation of an agent.
 */
class AgentPlan {
private:
    /* Serial ID */
    std::string id;

    /* Current position */
    multi_agent_planning::Position currentPos;

    /* Goal position */
    multi_agent_planning::Position goalPos;

    /* Planned path including the current and goal position */
    std::vector<multi_agent_planning::Position> path;

public:
    AgentPlan(std::string id, multi_agent_planning::Position pos);
    multi_agent_planning::Position getCurrentPos();
    multi_agent_planning::Position getGoalPos();
    std::vector<multi_agent_planning::Position> getPath();

    void setCurrentPos(multi_agent_planning::Position pos);
    void setGoalPos(multi_agent_planning::Position pos);
    void setPath(std::vector<multi_agent_planning::Position> path);

    std::string description();
};

#endif
