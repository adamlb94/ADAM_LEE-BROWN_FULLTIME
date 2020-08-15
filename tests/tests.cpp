#include "ros/ros.h"

#include <ros/console.h>
#include <gtest/gtest.h>

const std::string NODE_NAME = "test_multi_agent_planning";

TEST(Test1, basicTest) {
    EXPECT_TRUE(true);
}

TEST(Test2, basicTest) {
    EXPECT_TRUE(false);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}