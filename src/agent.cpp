#include "agent.h"
#include "multi_agent_planning/AgentPos.h"
#include "multi_agent_planning/GetPlan.h"

/* ROS node name */
#define NODE_NAME "agent"

/* The service for agent nodes to have their end goal specified */
#define UPDATE_GOAL_SERVICE "update_goal"

using namespace multi_agent_planning;

/**
 * Publish the agent's current position on the /agent_feedback topic.
 */
int Agent::publishPos() {
    AgentPos agentPos;
    agentPos.id = id;
    agentPos.position = pos;
    agentFeedbackPublisher.publish(agentPos);

    ros::spinOnce();
}

/**
 * Callback for UPDATE_GOAL_SERVICE.
 *
 * @param req the service request
 * @param res the service response
 */
bool Agent::updateGoalCallback(UpdateGoal::Request& req, UpdateGoal::Response &res) {
    goalPos.x = req.x;
    goalPos.y = req.y;
    goalPos.theta = req.theta;
    ROS_INFO("(updateGoalCallback) Updated goalPos: %d, %d, %d", goalPos.x, goalPos.y, goalPos.theta);

    res.result = true;

    endGoalReceived = true;
    return true;
}

/**
 * Requests a path plan from the planning node.
 */
void Agent::getPlan() {
    GetPlan srv;
    srv.request.id = id;
    srv.request.goalPos = goalPos;

    if (getPlanClient.call(srv)) {
        path = (std::vector<Position>)srv.response.path;

        ROS_INFO("(getPlan): Plan:");
        for (Position pos : path) {
            ROS_INFO("%d, %d, %d", pos.x, pos.y, pos.theta);
        }
    } else {
        ROS_ERROR("Failed to call service %s", GET_PLAN_SERVICE);
    }
}

/**
 * Initializes ROS components and sets the agent start position using the given command-line arguments.
 *
 * @param argc number of command-line arguments
 * @param argv command-line arguments
 * @return zero if initialization succeeded
 */
int Agent::init(int argc, char **argv) {
    /* Assign agent attributes */
    // TODO: input value checks
    id = argv[1];
    pos.x = atoi(argv[2]);
    pos.y = atoi(argv[3]);
    pos.theta = atoi(argv[4]);
    endGoalReceived = false;

    /* ROS initialization */
    ros::init(argc, argv, NODE_NAME);
    nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    agentFeedbackPublisher = nodeHandle->advertise<AgentPos>(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE);
    updateGoalServer = nodeHandle->advertiseService(id + "/" + UPDATE_GOAL_SERVICE, &Agent::updateGoalCallback, this);
    getPlanClient = nodeHandle->serviceClient<GetPlan>(GET_PLAN_SERVICE);

    return 0;
}

/**
 * Executes the agent.
 *
 * @return zero if execution completed successfully
 */
int Agent::execute() {
    while (0 == agentFeedbackPublisher.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
    }

    endGoalReceived = false;
    publishPos();

    /* Check for an end point on the UPDATE_GOAL_SERVICE at a rate of 10 Hz */
    ros::Rate r(10);
    while (!endGoalReceived) {
        if (!ros::ok()) {
            return 1;
        }
        ROS_INFO_ONCE("Waiting for %s", UPDATE_GOAL_SERVICE);
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Getting plan...");
    getPlan();

    ROS_INFO("Done");
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 5) {
        std::cout << "Required: agent <serial-id> <start-X> <start-Y> <start-theta> " << argc << std::endl;
        return 1;
    }

    Agent agent;
    int init_result = agent.init(argc, argv);
    if (init_result) {
        return init_result;
    }

    return agent.execute();
}
