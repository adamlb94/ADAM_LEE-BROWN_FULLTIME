#include "common.h"
#include <sstream>

const std::string NODE_NAME = "agent";

/**
 * Agent node which requests paths from the planner.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_NAME);
  if (argc != 5) {
    ROS_INFO("Required: agent <serial-id> <start-X> <start-Y> <start-theta>");
    return 1;
  }

  ros::NodeHandle n;

  /* Service */
  ros::ServiceClient client = n.serviceClient<multi_agent_planning::GetPlan>(GET_PLAN_SERVICE);

  multi_agent_planning::GetPlan srv;
  srv.request.serial_id = argv[1];
  srv.request.x = atoi(argv[2]);
  srv.request.y = atoi(argv[3]);
  srv.request.theta = atoi(argv[4]);

  if (client.call(srv)) {
    ROS_INFO("Result: %s", ((std::string)srv.response.result).c_str());
  } else {
    ROS_ERROR("Failed to call service %s", GET_PLAN_SERVICE);
    return 1;
  }

  /* Topic */
  ros::Publisher pub = n.advertise<multi_agent_planning::AgentPos>(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    multi_agent_planning::AgentPos pos;
    pos.x = count;
    pos.y = count;
    pos.theta = count;
    pub.publish(pos);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}