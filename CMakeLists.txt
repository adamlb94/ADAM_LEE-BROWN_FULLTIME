cmake_minimum_required(VERSION 3.0.2)
project(multi_agent_planning)

## Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    visualization_msgs
)

find_package(rostest REQUIRED)

add_message_files(
    FILES
    AgentPos.msg
    Position.msg
)

add_service_files(
    FILES
    GetPlan.srv
    UpdateGoal.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

## catkin specific configuration
catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(agent_plan src/agent_plan.cpp)
target_link_libraries(agent_plan ${catkin_LIBRARIES})

add_library(path_cache src/path_cache.cpp)
target_link_libraries(path_cache ${catkin_LIBRARIES})

add_library(roadmap src/roadmap.cpp)
target_link_libraries(roadmap ${catkin_LIBRARIES})

add_executable(agent src/agent.cpp)
target_link_libraries(agent ${catkin_LIBRARIES})
add_dependencies(agent multi_agent_planning_generate_messages_cpp)

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} agent_plan path_cache roadmap)
add_dependencies(planner multi_agent_planning_generate_messages_cpp agent_plan path_cache roadmap)

add_executable(test_multi_agent_planning tests/test.test tests/tests.cpp)
## add_rostest_gtest(test_multi_agent_planning tests/test.test tests/tests.cpp)
target_link_libraries(test_multi_agent_planning ${catkin_LIBRARIES} ${GTEST_LIBRARIES})