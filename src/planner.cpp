#include "common.h"

const std::string NODE_NAME = "planner";

/**
 * Planning node that plans paths for agents.
 */
class Planner {
private:
  std::unique_ptr<ros::NodeHandle> nodeHandle;
  ros::Subscriber agentFeedbackSubscriber;
  ros::ServiceServer getPlanServer;
  ros::ServiceServer updateGoalServer;

/**
 * Callback for the get_plan service.
 */
bool getPlanCallback(multi_agent_planning::GetPlan::Request  &req,
                     multi_agent_planning::GetPlan::Response &res) {
  res.result = true;

  ROS_INFO("getPlanCallback Agent: serial_id=%s, x=%d, y=%d, theta=%d", ((std::string)req.serial_id).c_str(), (int)req.x, (int)req.y, (int)req.theta);
  ROS_INFO("Response: [%d]", (bool)res.result);
  return true;
}

/**
 * Callback for the update_goal service.
 */
bool updateGoalCallback(multi_agent_planning::UpdateGoal::Request  &req,
                     multi_agent_planning::UpdateGoal::Response &res) {
  res.result = true;

  ROS_INFO("updateGoalCallback Agent: serial_id=%s, x=%d, y=%d, theta=%d", ((std::string)req.serial_id).c_str(), (int)req.x, (int)req.y, (int)req.theta);
  ROS_INFO("Response: [%d]", (bool)res.result);
  return true;
}

/**
 * Callback for the agent_feedback topic.
 */
void agentFeedbackCallback(const multi_agent_planning::AgentPos::ConstPtr& msg) {
  ROS_INFO("Agent pos: [%d, %d, %d]", msg->x, msg->y, msg->theta);
}

public:
  /**
   * Initializes ROS components.
   *
   * @param argc number of command-line arguments
   * @param argv command-line arguments
   */
  int init(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

    ROS_INFO("Ready to receive requests from agent.");
    getPlanServer = nodeHandle->advertiseService(GET_PLAN_SERVICE, &Planner::getPlanCallback, this);
    updateGoalServer = nodeHandle->advertiseService(UPDATE_GOAL_SERVICE, &Planner::updateGoalCallback, this);
    ROS_INFO("Subscribed.");
    agentFeedbackSubscriber = nodeHandle->subscribe(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE, &Planner::agentFeedbackCallback, this);
  }

  /**
   * Executes the planner.
   */
  int execute() {
    ros::spin();
  }

};


int main(int argc, char **argv) {
  Planner planner;
  planner.init(argc, argv);
  planner.execute();

  return 0;
}