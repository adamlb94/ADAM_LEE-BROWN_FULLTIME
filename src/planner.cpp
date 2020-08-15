#include "common.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <queue>
#include <functional> // for std::hash

#define WIDTH 11
#define HEIGHT 11

/* Minimum clearance between two agents (in seconds) */
#define MIN_CLEARANCE 10

using namespace multi_agent_planning;

const std::string NODE_NAME = "planner";

const int x_directions[] = {1, 0, -1, 0};
const int y_directions[] = {0, 1, 0, -1};

/**
 * Planning node's representation of an agent.
 */
class AgentPlan {
private:
    std::string id;
    Position currentPos;
    Position goalPos;
    /* Planned path, not including the current and goal position */
    std::vector<Position> path;

public:
    AgentPlan(std::string id, Position pos) {
        this->id = id;
        setCurrentPos(pos);
    }

    Position getCurrentPos() {
        return currentPos;
    }

    Position getGoalPos() {
        return goalPos;
    }

    std::string getId() {
        return id;
    }

    std::vector<Position> getPath() {
        return path;
    }

    void setCurrentPos(Position pos) {
        currentPos = pos;
    }

    void setGoalPos(Position pos) {
        goalPos = pos;
    }

    void setPath(std::vector<Position> path) {
        this->path = path;
    }

    std::string description() {
        std::ostringstream oss;
        oss << (std::string)id << " "
            << "Current pos: (" << (int)currentPos.x << "," << (int)currentPos.y << "," <<  (int)currentPos.theta << ") "
            << "Goal pos: (" << (int)goalPos.x << "," << (int)goalPos.y << "," <<  (int)goalPos.theta << ")";
        return oss.str();
    }
};

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

    Roadmap() {
        for (int x = 0; x < WIDTH; x++)  {
            for (int y = 0; y < HEIGHT; y++) {
                roadmap[x][y] = -1;
            }
        }
    }

    void init(std::unique_ptr<ros::NodeHandle> &nodeHandle) {
        gridMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("grid_marker_array", WIDTH * HEIGHT);
        pathMarkerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("path_marker_array", WIDTH * HEIGHT);
        pathLinePublisher = nodeHandle->advertise<visualization_msgs::Marker>("path_line_marker", WIDTH * HEIGHT * 4);
    }

    void set(int x, int y, int state) {
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
            roadmap[x][y] = state;
        }
    };

    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        int i = (x + (y * HEIGHT));
        addMarker(gridPointMarkers, i, "grid", x, y, 0.2, 0.2, 0.0, 0.0, 0.0);
    }

    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
        addMarker(gridPointMarkers, 0, agentId, x, y, 0.90, 0.90, 0.0, 0.0, 0.0);
    }

    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, std::string agentId, int x, int y) {
        addMarker(gridPointMarkers, 1, agentId, x, y, 0.90, 0.90, 0.0, 1.0, 0.0);
    }

    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int i, std::string ns, int x, int y, double xScale, double yScale, double red, double green, double blue) {
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

    void displayRoadmap() {

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
};

/**
 * Cache for planned paths.
 */
class PathCache {
private:
    /**
     * A 2D coordinate.
     */
    struct Point {
        int x;
        int y;

        /**
         * Define '<' operator to allow Points to be used as map keys.
         */
        bool operator<(const Point p) const {
            return x < p.x || (x == p.x && y < p.y);
        }
    };

    /* Paths cache. Structure: map<startPoint, map<endPoint, path>> */
    std::map<Point, std::map<Point, std::vector<Position>>> paths;

    /**
     * Returns the Point representation of the given Position.
     *
     * @param pos the Position
     * @return the Point representation
     */
    Point toPoint(Position pos) {
        Point p;
        p.x = pos.x;
        p.y = pos.y;
        return p;
    }

public:
    /**
     * Returns the shortest path of Positions between the given points, inclusive, if it exists. Returns empty path otherwise.
     *
     * @param startPos the start Position
     * @param endPos the end Position
     * @return the shortest path, if it exists
     */
    std::vector<Position> get(Position startPos, Position endPos) {
        std::vector<Position> path;

        Point start = toPoint(startPos);
        auto startIt = paths.find(start);
        if (startIt != paths.end()) {
            Point end = toPoint(endPos);

            auto endIt = startIt->second.find(end);
            if (endIt != startIt->second.end()) {
                ROS_INFO("(Cache.get) PATH EXISTS");
                path = endIt->second;
            }
        }
        return path;
    }

    /**
     * Add the given path to the path cache.
     *
     * @param path the path to add
     */
    void put(std::vector<Position> path) {
        int pathSize = path.size();
        if (pathSize < 2) {
            ROS_WARN("(Cache.put) Given path is too short");
            return;
        }

        Point start = toPoint(path.at(0));
        Point end = toPoint(path.at(pathSize - 1));

        auto startIt = paths.find(start);
        if (startIt != paths.end()) {
            /* Start point has been used before */
            auto endIt = startIt->second.find(end);
            if (endIt != startIt->second.end()) {
                /* This start-end path has been added before */
                if (pathSize < endIt->second.size()) {
                    /* New path is shorter, use it */
                    ROS_INFO("(Cache.put) Updating path");
                    endIt->second = path;
                }
            } else {
                /* Add end-path entry start point map */
                ROS_INFO("(Cache.put) Start point exists, adding new path");
                startIt->second.insert({end, path});
            }
        } else {
            /* New start point */
            std::map<Point, std::vector<Position>> endPathMap;
            endPathMap.insert({end, path});

            paths.insert({start, endPathMap});
            ROS_INFO("(Cache.put) Adding new path");
        }
    }

};

/**
 * Planning node that plans paths for agents.
 */
class Planner {
private:
    std::unique_ptr<ros::NodeHandle> nodeHandle;
    ros::Subscriber agentFeedbackSubscriber;
    ros::ServiceServer getPlanServer;

    std::unordered_map<std::string, AgentPlan> agents;

    Roadmap roadmap;
    PathCache pathCache;

    /**
     * Callback for the agent_feedback topic.
     */
    void agentFeedbackCallback(const AgentPos::ConstPtr& msg) {
        std::string id = (std::string)msg->id;
        Position pos = (Position)msg->position;

        auto it = agents.find(id);
        if (it == agents.end()) {
            /* First time hearing from this agent. */
            AgentPlan agent = AgentPlan(id, pos);
            agents.insert({id, agent});
            ROS_INFO("(agentFeedbackCallback) Created new agent: %s", agent.description().c_str());
        } else {
            it->second.setCurrentPos(pos);
            ROS_INFO("(agentFeedbackCallback) Updated agent: %s", it->second.description().c_str());
        }
    }

    Position position(int x, int y) {
        Position p;
        p.x = x;
        p.y = y;
        p.theta = 0;
        return p;
    }

    int getCost(int costs[WIDTH][HEIGHT], int startX, int startY) {
        int minCost = INT_MAX;
        for (int dir = 0; dir < sizeof(x_directions); dir++) {
            int x = startX + x_directions[dir];
            int y = startY + y_directions[dir];
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
                /* Out of bounds of the roadmap */
                continue;
            }
            minCost = std::min(minCost, costs[x][y]);
        }
        return minCost == INT_MAX ? minCost : minCost + 10;
    }

    /**
     * Find the shortest path between the given positions using the planPath algorithm, keeping track of previous nodes in the path.
     *
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @param prevPos 2D array of the same size as the roadmap, containing the previous position of a potential path for every position in the roadmap
     * @return true if there is a path between the current and goal positions, false otherwise
     */
    bool planPath(Position currentPos, Position goalPos, Position prevPos[WIDTH][HEIGHT])
    {
        std::queue<Position> queue;
        int costs[WIDTH][HEIGHT];

        for (int x = 0; x < WIDTH; x++) {
            for (int y = 0; y < HEIGHT; y++) {
                // roadmap.roadmap[x][y] = false; // TODO
                prevPos[x][y] = currentPos;
                costs[x][y] = INT_MAX;
            }
        }

        /* Insert start (currentPos) into queue */
        int x = currentPos.x;
        int y = currentPos.y;
        costs[x][y] = 0;
        queue.push(position(x, y));

        /* BFS traversal */
        while (!queue.empty()) {
            Position currPos = queue.front();
            queue.pop();

            /* Explore coordinates in above, below and to the sides of the current position */
            for (int dir = 0; dir < sizeof(x_directions); dir++) {
                x = currPos.x + x_directions[dir];
                y = currPos.y + y_directions[dir];

                if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
                    /* Out of bounds of the roadmap */
                    continue;
                }
                if (costs[x][y] == INT_MAX) {
                    int cost = getCost(costs, x, y);
                    costs[x][y] = cost;
                    if (roadmap.roadmap[x][y] != INT_MAX && std::abs(cost - roadmap.roadmap[x][y]) <= MIN_CLEARANCE) { // TODO: Fix this
                        continue;
                    }

                    prevPos[x][y] = currPos;
                    Position p = position(x, y);
                    if (p == goalPos) {
                        /* Reached the end */
                        return true;
                    }
                    queue.push(p);
                }
            }
        }

        /* No path between points */
        return false;
    }

    /**
     * Constructs the shortest path between the two given positions, using the given 2D array of previous positions.
     *
     * @param path the path to fill
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @param prevPos 2D array of the same size as the roadmap, containing the previous position of a potential path for every position in the roadmap
     * @return the shortest path
     */
    void constructPath(std::vector<Position> *path, Position currentPos, Position goalPos, Position prevPos[WIDTH][HEIGHT]) {
        /* Start from the goalPos and work back */
        int x = goalPos.x;
        int y = goalPos.y;

        path->push_back(goalPos);
        while (prevPos[x][y].x != currentPos.x || prevPos[x][y].y != currentPos.y) {
            Position prev = prevPos[x][y];
            path->push_back(prev);
            x = prev.x;
            y = prev.y;
        }
        path->push_back(currentPos);

        /* Return the path in the correct order */
        std::reverse(path->begin(), path->end());
    }

    /**
     * Finds the shortest path between the given positions. The given positions are NOT included in the path.
     *
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @return the shortest path
     */
    std::vector<Position> getShortestPath(Position currentPos, Position goalPos) {
        std::vector<Position> path = pathCache.get(currentPos, goalPos);
        if (!path.empty()) {
            ROS_INFO("(getShortestPath) Path already exists!");
            return path;
        }

        /* Previous position of a potential path for every position in the roadmap */
        Position prevPos[WIDTH][HEIGHT];

        if (planPath(currentPos, goalPos, prevPos) == false) {
            ROS_WARN("(getShortestPath) No path between points.");

            path.push_back(currentPos);
            return path;
        }

        /* Positions in the shortest path */
        constructPath(&path, currentPos, goalPos, prevPos);

        std::cout << "Shortest path: ";
        for (int i = 0; i < path.size(); i++) {
            Position p = path.at(i);
            std::cout << "(" << (int)p.x << "," << (int)p.y << "),";
            roadmap.set(p.x, p.y, i * 10);
        }
        std::cout << std::endl;

        /* Print entire roadmap, with path shown as true values */
        for (int row = 0; row < WIDTH; row++)  {
            for (int col = 0; col < HEIGHT; col++) {
                std::cout << roadmap.roadmap[row][col] << ", ";
            }
            std::cout << std::endl;
        }

        pathCache.put(path);
        return path;
    }

    /**
     * Callback for the get_plan service.
     */
    bool getPlanCallback(GetPlan::Request  & req,
                         GetPlan::Response & res) {
        std::string id = (std::string)req.id;
        auto it = agents.find(id);

        if (it == agents.end()) {
            ROS_WARN("(getPlanCallback) agent not found!");
            // TODO: Need to return message?
            return false;
        } else {
            it->second.setGoalPos(req.goalPos);
            ROS_INFO("(getPlanCallback) Updated agent: %s", it->second.description().c_str());
        }

        std::vector<Position> path = getShortestPath(it->second.getCurrentPos(), it->second.getGoalPos());
        it->second.setPath(path);

        roadmap.displayPath(id, path, nodeHandle);

        res.path = path;

        ROS_INFO("(getPlanCallback) Created plan for agent id=%s", ((std::string)req.id).c_str());
        return true;
    }

public:
    /**
     * Initializes ROS components.
     *
     * @param argc number of command-line arguments
     * @param argv command-line arguments
     */
    int init(int argc, char **argv) {
        ros::init(argc, argv, NODE_NAME);
        nodeHandle = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

        roadmap.init(nodeHandle);

        ROS_INFO("Ready to receive requests from agent.");
        agentFeedbackSubscriber = nodeHandle->subscribe(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE, &Planner::agentFeedbackCallback, this);
        getPlanServer = nodeHandle->advertiseService(GET_PLAN_SERVICE, &Planner::getPlanCallback, this);
        ROS_INFO("Subscribed.");
    }

    /**
     * Executes the planner.
     */
    int execute() {
        roadmap.displayRoadmap();

        ros::spin();
    }

};

int main(int argc, char **argv) {
    Planner planner;
    planner.init(argc, argv);
    planner.execute();

    return 0;
}