#include "roadmap.h"
#include "ros/ros.h"
#include <vector>

using namespace multi_agent_planning;

/**
 * Creates a std_msgs::ColorRGBA from the given RGB values.
 *
 * @param r red
 * @param g green
 * @param b blue
 * @return the std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA Roadmap::rgbColor(double r, double g, double b) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    return color;
}

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

    /* Set colors to be used by rviz for displaying agent paths */
    agentColorCount = 0;
    colors.push_back(rgbColor(1.0, 0.0, 0.0)); // Red
    colors.push_back(rgbColor(0.0, 1.0, 0.0)); // green
    colors.push_back(rgbColor(0.0, 0.0, 1.0)); // blue
    colors.push_back(rgbColor(1.0, 1.0, 0.0)); // yellow
    colors.push_back(rgbColor(0.0, 1.0, 1.0)); // cyan
    colors.push_back(rgbColor(0.0, 0.5, 0.0)); // dark green
    colors.push_back(rgbColor(0.0, 0.0, 0.5)); // dark blue
    colors.push_back(rgbColor(1.0, 0.0, 1.0)); // magenta
    colors.push_back(rgbColor(0.5, 0.0, 0.0)); // maroon
    colors.push_back(rgbColor(1.0, 1.0, 1.0)); // white

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
    addMarker(pointMarkers, i, "grid", x, y, 0.2, 0.2, rgbColor(0.0, 0.0, 0.0));
}

/**
 * Add an agent marker to the given marker array at the given x-y coordinate.
 *
 * @param pointMarkers the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 */
void Roadmap::addAgentMarker(visualization_msgs::MarkerArray *pointMarkers, std::string agentId, int x, int y) {
    addMarker(pointMarkers, 0, agentId, x, y, 0.90, 0.90, rgbColor(0.0, 0.0, 0.0));
}

/**
 * Add a goal marker to the given marker array at the given x-y coordinate.
 *
 * @param pointMarkers the marker array
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @param color the marker color
 */
void Roadmap::addGoalMarker(visualization_msgs::MarkerArray *pointMarkers, std::string agentId, int x, int y, std_msgs::ColorRGBA color) {
    addMarker(pointMarkers, 1, agentId, x, y, 0.90, 0.90, color);
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
 * @param color the marker color
 */
void Roadmap::addMarker(visualization_msgs::MarkerArray *pointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, std_msgs::ColorRGBA color) {
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
    pointMarkers->markers[i].scale.z = xScale;

    pointMarkers->markers[i].color = color;
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

    /* Publish the marker */
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

    std::string agentId = id;
    pathLineList.ns = "path" + agentId;

    pathLineList.pose.orientation.w = 1.0;
    pathLineList.id = 0;
    pathLineList.type = visualization_msgs::Marker::LINE_LIST;
    pathLineList.scale.x = 0.2;

    std_msgs::ColorRGBA color = colors.at(agentColorCount);
    agentColorCount = (agentColorCount + 1) % colors.size();
    pathLineList.color = color;

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
            addGoalMarker(&pathPointMarkers, agentId, pathPos.x, pathPos.y, color);
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
