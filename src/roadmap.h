#ifndef ROADMAP_H
#define ROADMAP_H

#include "ros/ros.h"
#include "multi_agent_planning/Position.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define WIDTH 11
#define HEIGHT 11

/**
 * Roadmap for use by multi-agent path planner.
 */
class Roadmap {
public:
    struct CellOccupation {
        /* Seconds from t0 at which an agent will be centered at a point */
        int time;
        /* Angle of the robot's movement from a point in degrees */
        int movementAngle;

        CellOccupation() {}
        CellOccupation(int time, int movementAngle);
    };

    void init(std::unique_ptr<ros::NodeHandle> &nodeHandle);
    void set(int x, int y, CellOccupation occupation);
    CellOccupation at(int x, int y);

    void displayRoadmap();
    void displayPath(std::string id, std::vector<multi_agent_planning::Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle);

private:
    CellOccupation roadmap[WIDTH][HEIGHT];

    ros::Publisher gridMarkerArrayPublisher;
    ros::Publisher pathMarkerArrayPublisher;
    ros::Publisher pathLinePublisher;

    std::vector<std_msgs::ColorRGBA> colors;
    int agentColorCount;

    std_msgs::ColorRGBA rgbColor(double r, double g, double b);
    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y);
    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y);
    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y, std_msgs::ColorRGBA color);
    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, std_msgs::ColorRGBA color);
};

#endif
