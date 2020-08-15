#include "agent.h"

int Agent::publishPos() {
    AgentPos agentPos;
    agentPos.id = id;
    agentPos.position = pos;
    agentFeedbackPublisher.publish(agentPos);

    ros::spinOnce();
}

/**
 * Callback for the update_goal service.
 */
bool Agent::updateGoalCallback(UpdateGoal::Request  & req,
                               UpdateGoal::Response & res) {
    goalPos.x = req.x;
    goalPos.y = req.y;
    goalPos.theta = req.theta;
    ROS_INFO("(updateGoalCallback) Updated goalPos: %d, %d, %d", goalPos.x, goalPos.y, goalPos.theta);

    res.result = true;

    waitingForResponse = false;
    return true;
}

/**
 * Requests a plan from the planning node.
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
 */
int Agent::init(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    if (argc != 5) {
        ROS_INFO("Required: agent <serial-id> <start-X> <start-Y> <start-theta>");
        return 1;
    }

    id = argv[1];
    pos.x = atoi(argv[2]);
    pos.y = atoi(argv[3]);
    pos.theta = atoi(argv[4]);

    nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    agentFeedbackPublisher = nodeHandle->advertise<AgentPos>(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE);
    updateGoalServer = nodeHandle->advertiseService(id + "/" + UPDATE_GOAL_SERVICE, &Agent::updateGoalCallback, this);
    getPlanClient = nodeHandle->serviceClient<GetPlan>(GET_PLAN_SERVICE);

    return 0;
}

/**
 * Executes the agent.
 */
int Agent::execute() {
    while (0 == agentFeedbackPublisher.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
    }

    publishPos();
    waitingForResponse = true;
    ros::Rate r(10); // 10 Hz
    while (waitingForResponse) {
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
    Agent agent;
    agent.init(argc, argv);
    agent.execute();

    return 0;
}