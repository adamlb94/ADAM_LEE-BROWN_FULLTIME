#include "roadmap.h"

/**
 * Roadmap for use by multi-agent path planner.
 */
class Roadmap {
private:
    ros::Publisher markerArrayPublisher;
public:
    int roadmap[HEIGHT][WIDTH]; // TODO: make private

    Roadmap() {
        for (int x = 0; x < HEIGHT; x++)  {
            for (int y = 0; y < WIDTH; y++) {
                roadmap[x][y] = -1;
            }
        }
    }

    void init(std::unique_ptr<ros::NodeHandle> &nodeHandle) {
        markerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("grid_marker_array", WIDTH * HEIGHT);
    }

    void set(int x, int y, int state) {
        if (x >= 0 && x < HEIGHT && y >= 0 && y < WIDTH) {
            roadmap[x][y] = state;
        }
    };

    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        addMarker(gridPointMarkers, "grid", x, y, 0.2, 0.2, 0.0, 0.0, 0.0);
    }

    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
        addMarker(gridPointMarkers, agentId, x, y, 0.90, 0.90, 0.0, 0.0, 1.0);
    }

    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
        addMarker(gridPointMarkers, agentId, x, y, 0.90, 0.90, 0.0, 1.0, 0.0);
    }

    void addPathMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
        addMarker(gridPointMarkers, agentId, x, y, 0.90, 0.90, 1.0, 0.0, 0.0);
    }

    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue) {
        int i = (x * WIDTH) + y;

        gridPointMarkers->markers[i].header.frame_id = "/roadmap";
        gridPointMarkers->markers[i].header.stamp = ros::Time::now();
        gridPointMarkers->markers[i].ns = ns;
        gridPointMarkers->markers[i].action = visualization_msgs::Marker::ADD;
        gridPointMarkers->markers[i].type = visualization_msgs::Marker::CYLINDER;
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
        gridPointMarkers->markers[i].scale.z = 0.1;

        gridPointMarkers->markers[i].color.a = 1.0;
        gridPointMarkers->markers[i].color.r = red;
        gridPointMarkers->markers[i].color.g = green;
        gridPointMarkers->markers[i].color.b = blue;
    }

    void displayRoadmap() {

        visualization_msgs::MarkerArray gridPointMarkers;
        gridPointMarkers.markers.resize(WIDTH * HEIGHT);

        for ( int x = 0; x < HEIGHT; x++) {
            for ( int y = 0; y < WIDTH; y++) {
                addGridPointMarker(&gridPointMarkers, x, y);
            }
        }

        // Publish the marker
        while (markerArrayPublisher.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN("Please create a subscriber to the marker");
            ros::Duration(0.5).sleep();
        }
        markerArrayPublisher.publish(gridPointMarkers);
    }

    /**
     * Returns an rviz namespace-appropriate namespace for the path of the agent with the given ID.
     */
    std::string toAlnum(std::string id) {
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

    void displayPath(std::string id, std::vector<Position> path, std::unique_ptr<ros::NodeHandle> &nodeHandle) {
        ros::Publisher markerPublisher = nodeHandle->advertise<visualization_msgs::Marker>("path_marker", WIDTH * HEIGHT);

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
        pathLineList.scale.x = 0.90;

        pathLineList.color.r = 1.0;
        pathLineList.color.a = 1.0;

        /* Points and lines */
        visualization_msgs::MarkerArray gridPointMarkers;
        gridPointMarkers.markers.resize(WIDTH * HEIGHT);

        for (int i = 0; i < path.size(); i++) {
            Position pathPos = path.at(i);

            geometry_msgs::Point p;
            p.x = pathPos.x;
            p.y = pathPos.y;
            p.z = 0.0;

            if (i == 0) {
                addAgentMarker(&gridPointMarkers, agentId, pathPos.x, pathPos.y);
            } else if (i == path.size() - 1) {
                addGoalMarker(&gridPointMarkers, agentId, pathPos.x, pathPos.y);
            } else {
                // addPathMarker(&gridPointMarkers, agentId, pathPos.x, pathPos.y);
                /* Add two points: the end of the prev line and the start of the next line */
                pathLineList.points.push_back(p);
            }
            pathLineList.points.push_back(p);
        }

        // Publish the marker
        while (markerArrayPublisher.getNumSubscribers() < 1 || markerPublisher.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return;
            }
            ROS_WARN("Please create a subscriber to the marker");
            ros::Duration(0.5).sleep();
        }
        markerArrayPublisher.publish(gridPointMarkers);
        markerPublisher.publish(pathLineList);
    }
};