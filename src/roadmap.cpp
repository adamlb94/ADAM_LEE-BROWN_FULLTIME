#include "roadmap.h"
#include "ros/ros.h"
#include <functional> // for std::hash
#include <vector>

using namespace multi_agent_planning;

/**
 * Constructor.
 *
 * @param nodeHandle the ROS node handle used for publishing to RVIZ
 * @param pos the agent's current position
 */
void Roadmap::init(std::unique_ptr<ros::NodeHandle> &nodeHandle) {
    for (int x = 0; x < WIDTH; x++)  {
        for (int y = 0; y < HEIGHT; y++) {
            roadmap[x][y] = -1;
        }
    }

    /* Initialize publishers */
    gridMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("grid_marker_array", WIDTH * HEIGHT);
    pathMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("path_marker_array", WIDTH * HEIGHT);
    pathLinePublisher = nodeHandle->advertise<visualization_msgs::Marker>("path_line_marker", WIDTH * HEIGHT * 4);
}

/**
 * Set the value of the given x-y coordinate.
 *
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param value the value to set
 */
void Roadmap::set(int x, int y, int value) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        roadmap[x][y] = value;
    }
};

/**
 * Add a grid marker to the given marker array at the given x-y coordinate.
 *
 * @param pointMarkers the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 */
void Roadmap::addGridPointMarker(visualization_msgs::MarkerArray *pointMarkers, int x, int y) {
    int i = (x + (y * HEIGHT));
    addMarker(pointMarkers, i, "grid", x, y, 0.2, 0.2, 0.0, 0.0, 0.0);
}

/**
 * Add an agent marker to the given marker array at the given x-y coordinate.
 *
 * @param pointMarkers the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 */
void Roadmap::addAgentMarker(visualization_msgs::MarkerArray *pointMarkers, std::string agentId, int x, int y) {
    addMarker(pointMarkers, 0, agentId, x, y, 0.90, 0.90, 0.0, 0.0, 0.0);
}

/**
 * Add a goal marker to the given marker array at the given x-y coordinate.
 *
 * @param pointMarkers the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 */
void Roadmap::addGoalMarker(visualization_msgs::MarkerArray *pointMarkers, std::string agentId, int x, int y) {
    addMarker(pointMarkers, 1, agentId, x, y, 0.90, 0.90, 0.0, 1.0, 0.0);
}

/**
 * Add a marker to the given marker array.
 *
 * @param pointMarkers the marker array
 * @param i the index to insert in the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param xScale the marker x-scale
 * @param yScale the marker y-scale
 * @param red the marker red color value
 * @param blue the marker blue color value
 * @param green the marker green color value
 */
void Roadmap::addMarker(visualization_msgs::MarkerArray *pointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue) {
    pointMarkers->markers[i].header.frame_id = "/roadmap";
    pointMarkers->markers[i].header.stamp = ros::Time::now();
    pointMarkers->markers[i].ns = ns;
    pointMarkers->markers[i].action = visualization_msgs::Marker::ADD;
    pointMarkers->markers[i].type = visualization_msgs::Marker::SPHERE;
    pointMarkers->markers[i].lifetime = ros::Duration();

    pointMarkers->markers[i].id = i;

    pointMarkers->markers[i].pose.orientation.x = 0.0;
    pointMarkers->markers[i].pose.orientation.y = 0.0;
    pointMarkers->markers[i].pose.orientation.z = 0.0;
    pointMarkers->markers[i].pose.orientation.w = 1.0;

    pointMarkers->markers[i].pose.position.x = x;
    pointMarkers->markers[i].pose.position.y = y;
    pointMarkers->markers[i].pose.position.z = 0;

    pointMarkers->markers[i].scale.x = xScale;
    pointMarkers->markers[i].scale.y = yScale;
    pointMarkers->markers[i].scale.z = xScale; // TODO

    pointMarkers->markers[i].color.a = 1.0;
    pointMarkers->markers[i].color.r = red;
    pointMarkers->markers[i].color.g = green;
    pointMarkers->markers[i].color.b = blue;
}

/**
 * Displays the roadmap (grid points only).
 */
void Roadmap::displayRoadmap() {
    visualization_msgs::MarkerArray gridPointMarkers;
    gridPointMarkers.markers.resize(WIDTH * HEIGHT);

    for ( int x = 0; x < WIDTH; x++) {
        for ( int y = 0; y < HEIGHT; y++) {
            addGridPointMarker(&gridPointMarkers, x, y);
        }
    }

    // Publish the marker
    while (gridMarkerArrayPublisher.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return;
        }
        ROS_WARN("Please create a subscriber to the marker");
        ros::Duration(0.5).sleep();
    }
    gridMarkerArrayPublisher.publish(gridPointMarkers);
}

/**
 * Returns the given string, stripped of any non-alphanumeric characters.
 * This creates an RVIZ namespace-appropriate namespace for the path of the agent with the given ID.
 *
 * @param s the string
 * @return a new string with any non-alphanumeric characters removed
 */
std::string Roadmap::toAlnum(std::string s) {
    std::string stripped = "";
    /* Remove all non-alphabetial characters from ID */
    for (int i = 0; i < s.length(); i++) {
        char c = s.at(i);
        if (isalnum(c)) {
            stripped = stripped + c;
        }
    }
    return stripped;
}

/**
 * Displays the path for an agent in RVIZ.
 *
 * @param id the agent's ID
 * @param path the path to display
 * @param s the string
 * @return nodeHandle the ROS node handle
 */
void Roadmap::displayPath(std::string id, std::vector<Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle) {
    /* Create marker array for start and end positions */
    visualization_msgs::MarkerArray pathPointMarkers;
    pathPointMarkers.markers.resize(2);

    /* Create marker for the path line */
    visualization_msgs::Marker pathLineList;

    pathLineList.header.frame_id = "/roadmap";
    pathLineList.header.stamp = ros::Time::now();
    pathLineList.action = visualization_msgs::Marker::ADD;

    std::string agentId = toAlnum(id);
    pathLineList.ns = "path" + agentId;

    pathLineList.pose.orientation.w = 1.0;
    pathLineList.id = 0;
    pathLineList.type = visualization_msgs::Marker::LINE_LIST;
    pathLineList.scale.x = 0.2;

    /* Generate line color from agent ID */
    std::hash<std::string> hasher;
    int hashed = std::abs((int) hasher(agentId));
    double r = 1 - (1 / (hashed % 10));
    hashed /= 10;
    double g = 1 / (hashed % 10);
    hashed /= 10;
    double b = 1 / (hashed % 10);

    pathLineList.color.r = r;
    pathLineList.color.g = g;
    pathLineList.color.b = b;
    pathLineList.color.a = 1.0;

    /* Add start/end markers and points for the path line */
    visualization_msgs::MarkerArray gridPointMarkers;
    gridPointMarkers.markers.resize(WIDTH * HEIGHT);
    for (int i = 0; i < path.size(); i++) {
        Position pathPos = path.at(i);

        geometry_msgs::Point p;
        p.x = pathPos.x;
        p.y = pathPos.y;
        p.z = 0.0;

        if (i == 0) {
            addAgentMarker(&pathPointMarkers, agentId, pathPos.x, pathPos.y);
        } else if (i == path.size() - 1) {
            addGoalMarker(&pathPointMarkers, agentId, pathPos.x, pathPos.y);
        } else {
            /* Add two points: the end of the prev line and the start of the next line */
            pathLineList.points.push_back(p);
        }
        pathLineList.points.push_back(p);
    }

    /* Publish the markers and path line */
    while (pathMarkerArrayPublisher.getNumSubscribers() < 1 || pathLinePublisher.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return;
        }
        ROS_WARN("Please create a subscriber to the marker");
        ros::Duration(0.5).sleep();
    }
    pathMarkerArrayPublisher.publish(pathPointMarkers);
    pathLinePublisher.publish(pathLineList);
}
