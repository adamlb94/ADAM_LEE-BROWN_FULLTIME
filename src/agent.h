#include "common.h"
#include <sstream>

using namespace multi_agent_planning;

#define NODE_NAME "agent"

/**
 * Agent node which requests paths from the planner.
 */
class Agent {
private:
    std::string id;
    Position pos;
    Position goalPos;
    std::vector<Position> path;

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
    bool updateGoalCallback(UpdateGoal::Request  & req, UpdateGoal::Response & res);

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
