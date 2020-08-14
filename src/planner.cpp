#include "common.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <queue>

#define WIDTH 11
#define HEIGHT 11

using namespace multi_agent_planning;

const std::string NODE_NAME = "planner";

const int x_directions[] = {1, 0, -1, 0};
const int y_directions[] = {0, 1, 0, -1};

/**
 * Planning node's representation of an agent.
 */
class Agent {
private:
    std::string id;
    Position currentPos;
    Position goalPos;
    /* Planned path, not including the current and goal position */
    std::vector<Position> path;

public:
    Agent(std::string id, Position pos) {
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
            << "Current pos: (" << (double)currentPos.x << "," << (double)currentPos.y << "," <<  (double)currentPos.theta << ") "
            << "Goal pos: (" << (double)goalPos.x << "," << (double)goalPos.y << "," <<  (double)goalPos.theta << ")";
        return oss.str();
    }
};

/**
 * Roadmap for use by multi-agent path planner.
 */
class Roadmap {
private:
public:
    bool roadmap[HEIGHT][WIDTH]; // TODO: make private

    Roadmap() {
        for (int x = 0; x < HEIGHT; x++)  {
            for (int y = 0; y < WIDTH; y++) {
                roadmap[x][y] = false;
            }
        }
    }

    void set(int x, int y, bool state) {
        if (x >= 0 && x < HEIGHT && y >= 0 && y < WIDTH) {
            roadmap[x][y] = state;
        }
    };

    void addGridPointMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        addMarker(gridPointMarkers, x, y, 0.2, 0.2, 0.0, 0.0, 0.0);
    }

    void addAgentMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        addMarker(gridPointMarkers, x, y, 0.90, 0.90, 0.0, 0.0, 1.0);
    }

    void addGoalMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        addMarker(gridPointMarkers, x, y, 0.90, 0.90, 0.0, 1.0, 0.0);
    }

    void addPathMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y) {
        addMarker(gridPointMarkers, x, y, 0.90, 0.90, 1.0, 0.0, 0.0);
    }

    void addMarker(visualization_msgs::MarkerArray *gridPointMarkers, int x, int y, double xScale, double yScale, double red, double green, double blue) {
        int i = (x * WIDTH) + y;

        gridPointMarkers->markers[i].header.frame_id = "/roadmap";
        gridPointMarkers->markers[i].header.stamp = ros::Time::now();
        gridPointMarkers->markers[i].ns = "roadmap";
        gridPointMarkers->markers[i].action = visualization_msgs::Marker::ADD;
        gridPointMarkers->markers[i].type = visualization_msgs::Marker::CYLINDER;
        gridPointMarkers->markers[i].lifetime = ros::Duration();

        gridPointMarkers->markers[i].id = i;

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

    void displayRoadmap(std::unique_ptr<ros::NodeHandle> &nodeHandle) {
        ros::Publisher markerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", WIDTH * HEIGHT);

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
            ROS_WARN_ONCE("Please create a subscriber to the marker");
        }
        markerArrayPublisher.publish(gridPointMarkers);
    }

    void displayPath(Position currentPos, std::vector<Position> path, Position goalPos, std::unique_ptr<ros::NodeHandle> &nodeHandle) {
        ros::Publisher markerArrayPublisher = nodeHandle->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", WIDTH * HEIGHT);
        ros::Publisher markerPublisher = nodeHandle->advertise<visualization_msgs::Marker>("visualization_marker", WIDTH * HEIGHT);

        visualization_msgs::Marker pathLineList;

        /* Path lines setup */
        pathLineList.header.frame_id = "/roadmap";
        pathLineList.header.stamp = ros::Time::now();
        pathLineList.action = visualization_msgs::Marker::ADD;
        pathLineList.ns = "path";
        pathLineList.pose.orientation.w = 1.0;
        pathLineList.id = 0;
        pathLineList.type = visualization_msgs::Marker::LINE_LIST;
        pathLineList.scale.x = 0.90;

        pathLineList.color.r = 1.0;
        pathLineList.color.a = 1.0;

        /* Points and lines */
        visualization_msgs::MarkerArray gridPointMarkers;
        gridPointMarkers.markers.resize(WIDTH * HEIGHT);
        addAgentMarker(&gridPointMarkers, currentPos.x, currentPos.y);
        geometry_msgs::Point agentPoint;
        agentPoint.x = currentPos.x;
        agentPoint.y = currentPos.y;
        agentPoint.z = 0.0;
        pathLineList.points.push_back(agentPoint);

        for (Position pathPos : path) {
            addPathMarker(&gridPointMarkers, pathPos.x, pathPos.y);
            geometry_msgs::Point p;
            p.x = pathPos.x;
            p.y = pathPos.y;
            p.z = 0.0;

            /* Add two points: the end of the prev line and the start of the next line */
            pathLineList.points.push_back(p);
            pathLineList.points.push_back(p);
        }

        addGoalMarker(&gridPointMarkers, goalPos.x, goalPos.y);
        geometry_msgs::Point goalPoint;
        goalPoint.x = goalPos.x;
        goalPoint.y = goalPos.y;
        goalPoint.z = 0.0;
        pathLineList.points.push_back(goalPoint);

        // Publish the marker
        while (markerArrayPublisher.getNumSubscribers() < 1 || markerPublisher.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
        }
        markerArrayPublisher.publish(gridPointMarkers);
        markerPublisher.publish(pathLineList);
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
    ros::ServiceServer updateGoalServer;

    std::unordered_map<std::string, Agent> agents;

    Roadmap roadmap;

    /**
     * Callback for the agent_feedback topic.
     */
    void agentFeedbackCallback(const AgentPos::ConstPtr& msg) {
        std::string id = (std::string)msg->id;
        Position pos = (Position)msg->position;

        auto it = agents.find(id);
        if (it == agents.end()) {
            /* First time hearing from this agent. */
            Agent agent = Agent(id, pos);
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

    /**
     * Find the shortest path between the given positions using the planPath algorithm, keeping track of previous nodes in the path.
     *
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @param prevPos 2D array of the same size as the roadmap, containing the previous position of a potential path for every position in the roadmap
     * @return true if there is a path between the current and goal positions, false otherwise
     */
    bool planPath(Position currentPos, Position goalPos, Position prevPos[HEIGHT][WIDTH])
    {
        std::queue<Position> queue;
        bool visited[HEIGHT][WIDTH];

        for (int x = 0; x < HEIGHT; x++) {
            for (int y = 0; y < WIDTH; y++) {
                roadmap.roadmap[x][y] = false; // TODO
                visited[x][y] = false;
                prevPos[x][y] = currentPos;
            }
        }

        /* Insert start (currentPos) into queue */
        int x = currentPos.x;
        int y = currentPos.y;
        visited[x][y] = true;
        queue.push(position(x, y));

        /* BFS traversal */
        while (!queue.empty()) {
            Position currPos = queue.front();
            queue.pop();

            /* Explore coordinates in above, below and to the sides of the current position */
            for (int dir = 0; dir < sizeof(x_directions); dir++) {
                x = currPos.x + x_directions[dir];
                y = currPos.y + y_directions[dir];

                if (x < 0 || x >= HEIGHT || y < 0 || y >= WIDTH) {
                    /* Out of bounds of the roadmap */
                    continue;
                }
                if (!visited[x][y] && !roadmap.roadmap[x][y]) {
                    visited[x][y] = true;
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
     * Returns the shortest path between the two given positions, using the given 2D array of previous positions.
     *
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @param prevPos 2D array of the same size as the roadmap, containing the previous position of a potential path for every position in the roadmap
     * @return the shortest path
     */
    std::vector<Position> constructPath(Position currentPos, Position goalPos, Position prevPos[HEIGHT][WIDTH]) {
        std::vector<Position> path;

        /* Start from the goalPos and work back */
        int x = goalPos.x;
        int y = goalPos.y;
        while (prevPos[x][y].x != currentPos.x || prevPos[x][y].y != currentPos.y) {
            Position prev = prevPos[x][y];
            path.push_back(prev);
            x = prev.x;
            y = prev.y;
        }

        /* Return the path in the correct order */
        std::reverse(path.begin(), path.end());
        return path;
    }

    /**
     * Finds the shortest path between the given positions. The given positions are NOT included in the path.
     *
     * @param currentPos the agent's current position
     * @param goalPos the agent's target position
     * @return the shortest path
     */
    std::vector<Position> getShortestPath(Position currentPos, Position goalPos) {
        std::vector<Position> path;
        /* Previous position of a potential path for every position in the roadmap */
        Position prevPos[HEIGHT][WIDTH];

        if (planPath(currentPos, goalPos, prevPos) == false) {
            ROS_WARN("(getShortestPath) No path between points.");
            return path;
        }

        /* Positions in the shortest path */
        path = constructPath(currentPos, goalPos, prevPos);

        std::cout << "Shortest path: ";
        for (Position p : path) {
            std::cout << "(" << p.x << "," << p.y << "),";
            roadmap.set(p.x, p.y, true); // TODO: path should be non-blocking
        }
        std::cout << std::endl;

        /* Print entire roadmap, with path shown as true values */
        for (int row = 0; row < HEIGHT; row++)  {
            for (int col = 0; col < WIDTH; col++) {
                std::cout << roadmap.roadmap[row][col] << ", ";
            }
            std::cout << std::endl;
        }

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
        }

        std::vector<Position> path = getShortestPath(it->second.getCurrentPos(), it->second.getGoalPos());
        // path.push_back(it->second.getCurrentPos()); // TODO
        // path.push_back(it->second.getGoalPos()); // TODO
        it->second.setPath(path);
        roadmap.displayRoadmap(nodeHandle);
        roadmap.displayPath(it->second.getCurrentPos(), path, it->second.getGoalPos(), nodeHandle);

        res.path = path;

        ROS_INFO("(getPlanCallback) Created plan for agent id=%s", ((std::string)req.id).c_str());
        return true;
    }

    /**
     * Callback for the update_goal service.
     */
    bool updateGoalCallback(UpdateGoal::Request  & req,
                            UpdateGoal::Response & res) {
        std::string id = (std::string)req.id;

        auto it = agents.find(id);
        if (it == agents.end()) {
            // TODO: handle if agent does not exist in map
        } else {
            it->second.setGoalPos(req.position);
            ROS_INFO("(updateGoalCallback) Updated agent: %s", it->second.description().c_str());
        }

        res.result = true;
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

        ROS_INFO("Ready to receive requests from agent.");
        getPlanServer = nodeHandle->advertiseService(GET_PLAN_SERVICE, &Planner::getPlanCallback, this);
        updateGoalServer = nodeHandle->advertiseService(UPDATE_GOAL_SERVICE, &Planner::updateGoalCallback, this);
        ROS_INFO("Subscribed.");
        agentFeedbackSubscriber = nodeHandle->subscribe(AGENT_FEEDBACK_TOPIC, QUEUE_SIZE, &Planner::agentFeedbackCallback, this);
    }

    /**
     * Executes the planner.
     */
    int execute() {
        ros::spin();
    }

};

int main(int argc, char **argv) {
    Planner planner;
    planner.init(argc, argv);
    planner.execute();

    return 0;
}