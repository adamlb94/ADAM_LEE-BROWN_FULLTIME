#ifndef ROADMAP_H
#define ROADMAP_H

#include "ros/ros.h"
#include "multi_agent_planning/Position.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define WIDTH 11 // TODO: remove, have as input args
#define HEIGHT 11 // TODO: remove, have as input args

/**
 * Roadmap for use by multi-agent path planner.
 */
class Roadmap {
private:
    ros::Publisher gridMarkerArrayPublisher;
    ros::Publisher pathMarkerArrayPublisher;
    ros::Publisher pathLinePublisher;

public:
    int roadmap[WIDTH][HEIGHT]; // TODO: make private

    void init(std::unique_ptr<ros::NodeHandle> &nodeHandle);
    void set(int x, int y, int value);

    void displayRoadmap();
    void displayPath(std::string id, std::vector<multi_agent_planning::Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle);

    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y);
    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y);
    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y);
    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue);

    std::string toAlnum(std::string id);
};

#endif
