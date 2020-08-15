#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "multi_agent_planning/Position.h"

#define WIDTH 11
#define HEIGHT 11 // TODO: remove

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

    void set(int x, int y, int state);

    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y);

    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y);

    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y);

    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue);

    void displayRoadmap();

    /**
     * Returns an rviz namespace-appropriate namespace for the path of the agent with the given ID.
     */
    std::string toAlnum(std::string id);

    void displayPath(std::string id, std::vector<multi_agent_planning::Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle);
};