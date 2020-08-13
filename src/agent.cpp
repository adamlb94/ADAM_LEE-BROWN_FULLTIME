#include "common.h"
#include <sstream>

const std::string NODE_NAME = "agent";

/**
 * Agent node which requests paths from the planner.
 */
class Agent {
private:
  /**
   * Agent position struct.
   */
  struct Position {
    int x;
    int y;
    int theta;
  };

  std::string serial_id;
  Position pos;

  std::unique_ptr<ros::NodeHandle> nodeHandle;
  ros::Publisher agentFeedbackPublisher;
  ros::ServiceClient getPlanClient;
  ros::ServiceClient updateGoalClient;

  /**
   * Publishes the agent's current position.
   */
  int publishPos() {
    multi_agent_planning::AgentPos agentPos;
    agentPos.x = pos.x;
    agentPos.y = pos.y;
    agentPos.theta = pos.theta;
    agentFeedbackPublisher.publish(agentPos);

    ros::spinOnce();
  }

  /**
   * Notifies the planning node of the agent's goal position.
   */
  int updateGoal() {
    multi_agent_planning::UpdateGoal srv;
    srv.request.serial_id = serial_id;
    srv.request.x = pos.x;
    srv.request.y = pos.y;
    srv.request.theta = pos.theta;

    if (updateGoalClient.call(srv)) {
      ROS_INFO("Result: %d", (bool)srv.response.result);
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
    srv.request.serial_id = serial_id;
    srv.request.x = pos.x;
    srv.request.y = pos.y;
    srv.request.theta = pos.theta;

    if (getPlanClient.call(srv)) {
      ROS_INFO("Result: %d", (bool)srv.response.result);
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

    serial_id = argv[1];
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
    publishPos();
    ros::Duration(1).sleep();

    updateGoal();
    ros::Duration(1).sleep();

    getPlan();

    return 0;
  }
};

int main(int argc, char **argv) {
  Agent agent;
  agent.init(argc, argv);
  agent.execute();

  return 0;
}