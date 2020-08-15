#include "common.h"
#include <sstream>

#define NODE_NAME "agent"

/**
 * Agent node which requests paths from the planner.
 */
class Agent {
private:
    std::string id;
    multi_agent_planning::Position pos;
    multi_agent_planning::Position goalPos;
    std::vector<multi_agent_planning::Position> path;

    std::unique_ptr<ros::NodeHandle> nodeHandle;
    ros::Publisher agentFeedbackPublisher;
    ros::ServiceServer updateGoalServer;
    ros::ServiceClient getPlanClient;

    bool waitingForResponse = false; // TODO: find alternative

    /**
     * Publishes the agent's current position.
     */
    int publishPos();

    /**
     * Callback for the get_plan service.
     */
    bool updateGoalCallback(multi_agent_planning::UpdateGoal::Request &req, multi_agent_planning::UpdateGoal::Response &res);

    /**
     * Requests a plan from the planning node.
     */
    void getPlan();

public:
    /**
     * Initializes ROS components and sets the agent start position using the given command-line arguments.
     *
     * @param argc number of command-line arguments
     * @param argv command-line arguments
     */
    int init(int argc, char **argv);

    /**
     * Executes the agent.
     */
    int execute();
};
