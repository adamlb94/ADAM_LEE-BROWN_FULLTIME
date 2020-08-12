#include "common.h"

const std::string NODE_NAME = "planner";

bool getPlanCallback(multi_agent_planning::GetPlan::Request  &req,
         multi_agent_planning::GetPlan::Response &res) {
  int sum = req.x + req.y + req.theta;
  std::stringstream ss;
  ss << req.serial_id << ", " << std::to_string(sum);
  res.result = ss.str();

  ROS_INFO("Agent: serial_id=%s, x=%d, y=%d, theta=%d", ((std::string)req.serial_id).c_str(), (int)req.x, (int)req.y, (int)req.theta);
  ROS_INFO("Response: [%s]", ((std::string)res.result).c_str());
  return true;
}

void agentFeedbackCallback(const multi_agent_planning::AgentPos::ConstPtr& msg) {
  ROS_INFO("Agent pos: [%d, %d, %d]", msg->x, msg->y, msg->theta);
}

/**
 * Planning node which plans paths for agents.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;

  /* Service */
  ros::ServiceServer service = n.advertiseService(GET_PLAN_SERVICE, getPlanCallback);
  ROS_INFO("Ready to receive requests from agent.");

  /* Topic */
  ros::Subscriber sub = n.subscribe(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE, agentFeedbackCallback);

  ros::spin();

  return 0;
}