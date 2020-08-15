#include "roadmap.h"
#include <functional> // for std::hash

using namespace multi_agent_planning;

void Roadmap::init(std::unique_ptr<ros::NodeHandle> &nodeHandle) {
    for (int x = 0; x < WIDTH; x++)  {
        for (int y = 0; y < HEIGHT; y++) {
            roadmap[x][y] = -1;
        }
    }

    gridMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("grid_marker_array", WIDTH * HEIGHT);
    pathMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("path_marker_array", WIDTH * HEIGHT);
    pathLinePublisher = nodeHandle->advertise<visualization_msgs::Marker>("path_line_marker", WIDTH * HEIGHT * 4);
}

void Roadmap::set(int x, int y, int state) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
        roadmap[x][y] = state;
    }
};

void Roadmap::addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
    int i = (x + (y * HEIGHT));
    addMarker(gridPointMarkers, i, "grid", x, y, 0.2, 0.2, 0.0, 0.0, 0.0);
}

void Roadmap::addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
    addMarker(gridPointMarkers, 0, agentId, x, y, 0.90, 0.90, 0.0, 0.0, 0.0);
}

void Roadmap::addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
    addMarker(gridPointMarkers, 1, agentId, x, y, 0.90, 0.90, 0.0, 1.0, 0.0);
}

void Roadmap::addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue) {
    gridPointMarkers->markers[i].header.frame_id = "/roadmap";
    gridPointMarkers->markers[i].header.stamp = ros::Time::now();
    gridPointMarkers->markers[i].ns = ns;
    gridPointMarkers->markers[i].action = visualization_msgs::Marker::ADD;
    gridPointMarkers->markers[i].type = visualization_msgs::Marker::SPHERE;
    gridPointMarkers->markers[i].lifetime = ros::Duration();

    gridPointMarkers->markers[i].id = i;

    gridPointMarkers->markers[i].pose.orientation.x = 0.0;
    gridPointMarkers->markers[i].pose.orientation.y = 0.0;
    gridPointMarkers->markers[i].pose.orientation.z = 0.0;
    gridPointMarkers->markers[i].pose.orientation.w = 1.0;

    gridPointMarkers->markers[i].pose.position.x = x;
    gridPointMarkers->markers[i].pose.position.y = y;
    gridPointMarkers->markers[i].pose.position.z = 0;

    gridPointMarkers->markers[i].scale.x = xScale;
    gridPointMarkers->markers[i].scale.y = yScale;
    gridPointMarkers->markers[i].scale.z = xScale; // TODO

    gridPointMarkers->markers[i].color.a = 1.0;
    gridPointMarkers->markers[i].color.r = red;
    gridPointMarkers->markers[i].color.g = green;
    gridPointMarkers->markers[i].color.b = blue;
}

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
 * Returns an rviz namespace-appropriate namespace for the path of the agent with the given ID.
 */
std::string Roadmap::toAlnum(std::string id) {
    std::string agentId = "";
    /* Remove all non-alphabetial characters from ID */
    for (int i = 0; i < id.length(); i++) {
        char c = id.at(i);
        if (isalnum(c)) {
            agentId = agentId + c;
        }
    }
    return agentId;
}

void Roadmap::displayPath(std::string id, std::vector<Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle) {
    visualization_msgs::MarkerArray pathPointMarkers;
    pathPointMarkers.markers.resize(2);

    visualization_msgs::Marker pathLineList;

    /* Path lines setup */
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
