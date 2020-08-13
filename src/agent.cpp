#include "common.h"
#include <sstream>

const std::string NODE_NAME = "agent";

/**
 * Agent node which requests paths from the planner.
 */
class Agent {
private:
    std::string id;
    multi_agent_planning::Position pos;
    std::vector<multi_agent_planning::Position> path;

    std::unique_ptr<ros::NodeHandle> nodeHandle;
    ros::Publisher agentFeedbackPublisher;
    ros::ServiceClient getPlanClient;
    ros::ServiceClient updateGoalClient;

    /**
     * Publishes the agent's current position.
     */
    int publishPos() {
        multi_agent_planning::AgentPos agentPos;
        agentPos.id = id;
        agentPos.position = pos;
        agentFeedbackPublisher.publish(agentPos);

        ros::spinOnce();
    }

    /**
     * Notifies the planning node of the agent's goal position.
     */
    int updateGoal() {
        multi_agent_planning::UpdateGoal srv;
        srv.request.id = id;
        srv.request.position = pos;

        if (updateGoalClient.call(srv)) {
            ROS_INFO("(updateGoal) result: %f", (bool)srv.response.result);
        } else {
            ROS_ERROR("Failed to call service %s", UPDATE_GOAL_SERVICE);
            return 1;
        }
    }

    /**
     * Requests a plan from the planning node.
     */
    int getPlan() {
        multi_agent_planning::GetPlan srv;
        srv.request.id = id;

        if (getPlanClient.call(srv)) {
            path = (std::vector<multi_agent_planning::Position>)srv.response.path;

            ROS_INFO("(getPlan): Plan:");
            for (multi_agent_planning::Position pos : path) {
                ROS_INFO("%f, %f, %f", pos.x, pos.y, pos.theta);
            }
        } else {
            ROS_ERROR("Failed to call service %s", GET_PLAN_SERVICE);
            return 1;
        }
    }

public:
    /**
     * Initializes ROS components and sets the agent start position using the given command-line arguments.
     *
     * @param argc number of command-line arguments
     * @param argv command-line arguments
     */
    int init(int argc, char **argv) {
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
        agentFeedbackPublisher = nodeHandle->advertise<multi_agent_planning::AgentPos>(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE);
        getPlanClient = nodeHandle->serviceClient<multi_agent_planning::GetPlan>(GET_PLAN_SERVICE);
        updateGoalClient = nodeHandle->serviceClient<multi_agent_planning::UpdateGoal>(UPDATE_GOAL_SERVICE);

        return 0;
    }

    /**
     * Executes the agent.
     */
    int execute() {
        while (ros::ok()) {
            publishPos();
            ros::Duration(1).sleep();

            updateGoal();
            ros::Duration(1).sleep();

            getPlan();
            ros::Duration(1).sleep();
        }
        return 0;
    }
};

int main(int argc, char **argv) {
    Agent agent;
    agent.init(argc, argv);
    agent.execute();

    return 0;
}