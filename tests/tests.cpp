#include "ros/ros.h"
#include "multi_agent_planning/AgentPos.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/Position.h"
#include "multi_agent_planning/UpdateGoal.h"

#include <ros/console.h>
#include <gtest/gtest.h>

#define AGENT_FEEDBACK_TOPIC "agent_feedback"
#define GET_PLAN_SERVICE "get_plan"
#define UPDATE_GOAL_SERVICE "update_goal"
#define NODE_NAME "test_multi_agent_planning"
#define QUEUE_SIZE 1000

using namespace multi_agent_planning;

std::unique_ptr<ros::NodeHandle> nodeHandle;
ros::Publisher agentFeedbackPublisher;
ros::ServiceServer updateGoalServer;
ros::ServiceClient getPlanClient;

void publishPos(std::string agentId, Position pos) {
    AgentPos agentPos;
    agentPos.id = agentId;
    agentPos.position = pos;
    agentFeedbackPublisher.publish(agentPos);

    ros::spinOnce();
}

std::vector<Position> getPlan(std::string agentId, Position endPos) {
    GetPlan srv;
    srv.request.id = agentId;
    srv.request.goalPos = endPos;

    std::vector<Position> path;
    if (getPlanClient.call(srv)) {
        path = (std::vector<Position>)srv.response.path;
    } else {
        ROS_ERROR("Failed to call service %s", GET_PLAN_SERVICE);
    }
    return path;
}

bool positionAt(Position p, int x, int y, int theta) {
    return p.x == x && p.y == y && p.theta == theta;
}

bool positionsMatch(Position p1, Position p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.theta == p2.theta;
}

bool pathsMatch(std::vector<Position> path1, std::vector<Position> path2) {
    if (path1.size() != path2.size()) {
        return false;
    }

    for (int i = 0; i < path1.size(); i++) {
        if (!positionsMatch(path1.at(i), path2.at(i))) {
            return false;
        }
    }
    return true;
}
Position position(int x, int y, int theta) {
    Position p;
    p.x = x;
    p.y = y;
    p.theta = theta;
    return p;
}

void testPath(std::string agentId, Position startPos, Position endPos, std::vector<Position> expectedPath) {
    nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
    agentFeedbackPublisher = nodeHandle->advertise<AgentPos>(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE);
    getPlanClient = nodeHandle->serviceClient<GetPlan>(GET_PLAN_SERVICE);

    int count = 10;
    while (count-- > 0 && 0 == agentFeedbackPublisher.getNumSubscribers()) {
        ROS_INFO("Waiting for subscribers to connect");
        ros::Duration(0.1).sleep();
    }

    publishPos(agentId, startPos);
    ros::Duration(0.5).sleep();
    std::vector<Position> path = getPlan(agentId, endPos);

    EXPECT_TRUE(pathsMatch(path, expectedPath));
}

// TEST(Test1, oneAgentShortPath) {
//     std::string agentId = "agent_1";
//     Position startPos = position(2, 0, 0);
//     Position endPos = position(2, 5, 0);

//     std::vector<Position> expectedPath;
//     expectedPath.push_back(position(2, 0, 0));
//     expectedPath.push_back(position(2, 1, 0));
//     expectedPath.push_back(position(2, 2, 0));
//     expectedPath.push_back(position(2, 3, 0));
//     expectedPath.push_back(position(2, 4, 0));
//     expectedPath.push_back(position(2, 5, 0));

//     testPath(agentId, startPos, endPos, expectedPath);
// }

TEST(Test1, oneAgentLongestPath) {
    std::string agentId = "agent_1";
    Position startPos = position(0, 0, 0);
    Position endPos = position(10, 10, 0);

    std::vector<Position> expectedPath;
    expectedPath.push_back(position(0, 0, 0));
    expectedPath.push_back(position(1, 0, 0));
    expectedPath.push_back(position(2, 0, 0));
    expectedPath.push_back(position(3, 0, 0));
    expectedPath.push_back(position(4, 0, 0));
    expectedPath.push_back(position(5, 0, 0));
    expectedPath.push_back(position(6, 0, 0));
    expectedPath.push_back(position(7, 0, 0));
    expectedPath.push_back(position(8, 0, 0));
    expectedPath.push_back(position(9, 0, 0));
    expectedPath.push_back(position(10, 0, 0));
    expectedPath.push_back(position(10, 1, 0));
    expectedPath.push_back(position(10, 2, 0));
    expectedPath.push_back(position(10, 3, 0));
    expectedPath.push_back(position(10, 4, 0));
    expectedPath.push_back(position(10, 5, 0));
    expectedPath.push_back(position(10, 6, 0));
    expectedPath.push_back(position(10, 7, 0));
    expectedPath.push_back(position(10, 8, 0));
    expectedPath.push_back(position(10, 9, 0));
    expectedPath.push_back(position(10, 10, 0));

    testPath(agentId, startPos, endPos, expectedPath);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}