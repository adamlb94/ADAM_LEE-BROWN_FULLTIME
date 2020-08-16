#include "multi_agent_planning/AgentPos.h"
#include "multi_agent_planning/GetPlan.h"
#include "multi_agent_planning/Position.h"
#include "multi_agent_planning/UpdateGoal.h"
#include "src/planner.h"
#include "src/agent.h"
#include "ros/ros.h"

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

/**
 * Publishes the start position for a mocked agent with the given ID.
 *
 * @param agentId the ID of the mocked agent
 * @param endPos the start position of the mocked agent
 */
void publishPos(std::string agentId, Position pos) {
    AgentPos agentPos;
    agentPos.id = agentId;
    agentPos.position = pos;
    agentFeedbackPublisher.publish(agentPos);

    ros::spinOnce();
}

/**
 * Retrieves a planned path from the planner node for a mocked agent with the given ID and end position (the start position having been provided prior).
 *
 * @param agentId the ID of the mocked agent
 * @param endPos the end position of the mocked agent
 * @return the path returned by the planner
 */
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

/**
 * Returns true if the given Position has the given coordinates.
 *
 * @param p the Position
 * @param x the expected x-coordinate
 * @param y the expected y-coordinate
 * @param theta expected the theta value
 * @return true if the position matches the given coordinates
 */
bool positionIs(Position p, int x, int y, int theta) {
    return p.x == x && p.y == y && p.theta == theta;
}

/**
 * Returns true if the given positions are equal.
 *
 * @param path1 the first position
 * @param path2 the second position
 * @return true if the positions match
 */
bool positionsMatch(Position p1, Position p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.theta == p2.theta;
}

/**
 * Returns true if the given paths consist of the same Positions.
 *
 * @param path1 the first path
 * @param path2 the second path
 * @return true if the paths match
 */
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

/**
 * Creates a Position from the given coordinates.
 *
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param theta the theta value
 * @return the Position
 */
Position position(int x, int y, int theta) {
    Position p;
    p.x = x;
    p.y = y;
    p.theta = theta;
    return p;
}

/**
 * Creates a mock agent with the given start position, end position, and expected shortest path, and queries the planner node.
 *
 * @param startPos the agent's start position
 * @param endPos the agent's end position
 * @param expectedPath the path that is expected to be computed by the planner
 * @return true if the path returns by the planner matches the expected path
 */
bool testPath(std::string agentId, Position startPos, Position endPos, std::vector<Position> expectedPath) {
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

    return pathsMatch(path, expectedPath);
}

/**
 * Required test, using mocked agents.
 */
TEST(Test, twoAgentsWithIntersectingPaths) {
    std::string agentId1 = "agent_1";
    Position startPos1 = position(2, 0, 0);
    Position endPos1 = position(2, 5, 0);

    std::vector<Position> expectedPath1;
    expectedPath1.push_back(position(2, 0, 0));
    expectedPath1.push_back(position(2, 1, 0));
    expectedPath1.push_back(position(2, 2, 0));
    expectedPath1.push_back(position(2, 3, 0));
    expectedPath1.push_back(position(2, 4, 0));
    expectedPath1.push_back(position(2, 5, 0));

    std::string agentId2 = "agent_2";
    Position startPos2 = position(0, 3, 0);
    Position endPos2 = position(6, 3, 0);

    std::vector<Position> expectedPath2;
    expectedPath2.push_back(position(0, 3, 0));
    expectedPath2.push_back(position(1, 3, 0));
    expectedPath2.push_back(position(1, 2, 0));
    expectedPath2.push_back(position(1, 1, 0));
    expectedPath2.push_back(position(2, 1, 0));
    expectedPath2.push_back(position(3, 1, 0));
    expectedPath2.push_back(position(4, 1, 0));
    expectedPath2.push_back(position(5, 1, 0));
    expectedPath2.push_back(position(5, 2, 0));
    expectedPath2.push_back(position(5, 3, 0));
    expectedPath2.push_back(position(6, 3, 0));

    EXPECT_TRUE(testPath(agentId1, startPos1, endPos1, expectedPath1) &&
                testPath(agentId2, startPos2, endPos2, expectedPath2));
}


int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}