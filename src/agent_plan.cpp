#include "agent_plan.h"

using namespace multi_agent_planning;

/**
 * Constructor.
 *
 * @param id the agent's ID
 * @param pos the agent's current position
 */
AgentPlan::AgentPlan(std::string id, Position pos) {
    this->id = id;
    setCurrentPos(pos);
}

/**
 * Return the agent's current position.
 *
 * @return the agent's current position
 */
Position AgentPlan::getCurrentPos() {
    return currentPos;
}

/**
 * Return the agent's goal position.
 *
 * @return the agent's goal position
 */
Position AgentPlan::getGoalPos() {
    return goalPos;
}

/**
 * Return the agent's planned path.
 *
 * @return the agent's path
 */
std::vector<Position> AgentPlan::getPath() {
    return path;
}

/**
 * Set the agent's current position.
 *
 * @param the agent's current position
 */
void AgentPlan::setCurrentPos(Position pos) {
    currentPos = pos;
}

/**
 * Set the agent's goal position.
 *
 * @param the agent's goal position
 */
void AgentPlan::setGoalPos(Position pos) {
    goalPos = pos;
}

/**
 * Set the agent's planned path.
 *
 * @param the agent's path
 */
void AgentPlan::setPath(std::vector<Position> path) {
    this->path = path;
}

/**
 * Returns a description of the agent (ID, current position, goal position).
 *
 * @param the agent's goal position
 */
std::string AgentPlan::description() {
    std::ostringstream oss;
    oss << (std::string)id << " "
        << "Current pos: (" << (int) currentPos.x << "," << (int) currentPos.y << "," <<  (int) currentPos.theta << ") "
        << "Goal pos: (" << (int) goalPos.x << "," << (int) goalPos.y << "," <<  (int) goalPos.theta << ")";
    return oss.str();
}
