#include "planner.h"

/* ROS node name */
#define NODE_NAME "planner"

/* Minimum clearance between two agents (in seconds) */
#define MIN_CLEARANCE 10

using namespace multi_agent_planning;

const int x_directions[] = {1, 0, -1, 0};
const int y_directions[] = {0, 1, 0, -1};

/**
 * Callback for AGENT_FEEDBACK_TOPIC.
 *
 * @param msg the ROS message
 */
void Planner::agentFeedbackCallback(const AgentPos::ConstPtr& msg) {
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

/**
 * Create a Position from the given x-y coordinates.
 *
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @return the Position
 */
Position Planner::position(int x, int y) {
    Position p;
    p.x = x;
    p.y = y;
    p.theta = 0;
    return p;
}

/**
 * Return the roadmap value at the given x-y coordinates.
 *
 * @param roadmap the roadmap
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @return the roadmap value
 */
int Planner::getValue(int roadmap[WIDTH][HEIGHT], int startX, int startY) {
    int minCost = INT_MAX;
    for (int dir = 0; dir < sizeof(x_directions); dir++) {
        int x = startX + x_directions[dir];
        int y = startY + y_directions[dir];
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
            /* Out of bounds of the roadmap */
            continue;
        }
        minCost = std::min(minCost, roadmap[x][y]);
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
bool Planner::planPath(Position currentPos, Position goalPos, Position prevPos[WIDTH][HEIGHT]) {
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
                int cost = getValue(costs, x, y);
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
void Planner::constructPath(std::vector<Position> *path, Position currentPos, Position goalPos, Position prevPos[WIDTH][HEIGHT]) {
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
std::vector<Position> Planner::getShortestPath(Position currentPos, Position goalPos) {
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
 * Callback for GET_PLAN_SERVICE.
 *
 * @param req the request
 * @param res the response
 */
bool Planner::getPlanCallback(GetPlan::Request &req, GetPlan::Response &res) {
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

/**
 * Initializes ROS components.
 *
 * @param argc number of command-line arguments
 * @param argv command-line arguments
 */
void Planner::init(int argc, char **argv) {
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
void Planner::execute() {
    roadmap.displayRoadmap();
    ros::spin();
}

int main(int argc, char **argv) {
    Planner planner;
    planner.init(argc, argv);
    planner.execute();
    return 0;
}
