#include "planner.h"

/* ROS node name */
#define NODE_NAME "planner"

/* Minimum clearance between two agents (in seconds) */
#define MIN_CLEARANCE 10

/* Cost of moving between roadmap coorinates (in seconds) */
#define COST_BETWEEN_COORDS 10

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
 * Return the minimum cost of the grid cell at the given x-y coordinates based on its surrounding cells.
 *
 * @param costs the roadmap
 * @param x the x-coordinate
 * @param y the y-coordinate
 * @return the cost
 */
int Planner::getValue(std::vector<std::vector<int>> costs, int startX, int startY) {
    int minCost = INT_MAX;
    for (int dir = 0; dir < sizeof(x_directions); dir++) {
        int x = startX + x_directions[dir];
        int y = startY + y_directions[dir];
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
            /* Out of bounds of the grid */
            continue;
        }
        minCost = std::min(minCost, costs.at(x).at(y));
    }
    return minCost == INT_MAX ? minCost : minCost + COST_BETWEEN_COORDS;
}

/**
 * Bi-directional BFS helper function: Explore the grid positions surrounding the given pos to find an intersection with the BFS search of the opposite direction.
 *
 * @param pos the centre position to explore around
 * @param endPos the goal position, return true if found
 * @param bfsFromStartCosts costs of each coordinate when searching for the endPos from the startPos
 * @param bfsFromEndCosts costs of each coordinate when searching for the startPos from the endPos
 * @param prevPos links to adjacent coorinates in potential paths
 * @param queue the BFS queue
 * @param intersectingPos position to set when an intersection between the two BFS directional searches are found
 * @return true if an intersection is found
 */
bool Planner::exploreCoord(Position pos, Position endPos, std::vector<std::vector<int>> *bfsFromStartCosts, std::vector<std::vector<int>> *bfsFromEndCosts, std::vector<std::vector<Position>> *prevPos, std::queue<Position> *queue, Position *intersectingPos) {
    for (int dir = 0; dir < sizeof(x_directions); dir++) {
        int x = pos.x + x_directions[dir];
        int y = pos.y + y_directions[dir];

        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
            /* Out of bounds of the roadmap */
            continue;
        }
        if (bfsFromStartCosts->at(x).at(y) == INT_MAX) {
            int cost = getValue(*bfsFromStartCosts, x, y);
            bfsFromStartCosts->at(x).at(y) = cost;

            if (roadmap.roadmap[x][y] != INT_MAX && std::abs(cost - roadmap.roadmap[x][y]) <= MIN_CLEARANCE) { // TODO: improve based on theta
                continue;
            }

            prevPos->at(x).at(y) = pos;
            if (bfsFromEndCosts->at(x).at(y) != INT_MAX) {
                *intersectingPos = position(x, y);
                return true;
            }

            Position p = position(x, y);
            if (p == endPos) {
                /* Reached the end */
                return true;
            }
            queue->push(p);
        }
    }
    return false;
}

/**
 * Find the shortest path between the given positions using a bi-directional BFS search algorithm.
 *
 * @param startPos the agent's start position
 * @param goalPos the agent's goal position
 * @param prevPos links to adjacent coorinates in potential paths beginning at the agent's start position
 * @param nextPos links to adjacent coorinates in potential paths beginning at the agent's goal position
 * @param intersectingPos position to set when an intersection between the two BFS directional searches are found
 * @return true if a path exists
 */
bool Planner::planPath(Position startPos, Position goalPos, std::vector<std::vector<Position>> *prevPos, std::vector<std::vector<Position>> *nextPos, Position *intersectingPos) {
    std::queue<Position> bfsFromStartQueue;
    std::vector<std::vector<int>> bfsFromStartCosts(WIDTH, std::vector<int>(HEIGHT, INT_MAX));

    std::queue<Position> bfsFromEndQueue;
    std::vector<std::vector<int>> bfsFromEndCosts(WIDTH, std::vector<int>(HEIGHT, INT_MAX));

    /* Insert startPos into bfsFromStartQueue */
    bfsFromStartCosts.at(startPos.x).at(startPos.y) = 0;
    bfsFromStartQueue.push(startPos);

    /* Insert endPos into bfsFromEndQueue */
    bfsFromEndCosts.at(goalPos.x).at(goalPos.y) = 0;
    bfsFromEndQueue.push(goalPos);

    /* BFS traversal */
    while (!bfsFromStartQueue.empty() && !bfsFromEndQueue.empty()) {
        Position bfsFromStartCurrPos = bfsFromStartQueue.front();
        bfsFromStartQueue.pop();

        if (exploreCoord(bfsFromStartCurrPos, goalPos, &bfsFromStartCosts, &bfsFromEndCosts, prevPos, &bfsFromStartQueue, intersectingPos)) {
            return true;
        }

        Position bfsFromEndCurrPos = bfsFromEndQueue.front();
        bfsFromEndQueue.pop();

        if (exploreCoord(bfsFromEndCurrPos, startPos, &bfsFromEndCosts, &bfsFromStartCosts, nextPos, &bfsFromEndQueue, intersectingPos)) {
            return true;
        }
    }

    /* No path between points */
    return false;
}

/**
 * Constructs the shortest path between the two given positions, meeting at the given intersection position, using the given 2D vectors of previous/next positions created during the bi-directional BFS.
 *
 * @param path the path to fill
 * @param startPos the agent's start position
 * @param goalPos the agent's target position
 * @param intersectionPos the position at which the bi-drectional BFS
 * @param prevPos links to adjacent coorinates in potential paths beginning at the agent's start position
 * @param nextPos links to adjacent coorinates in potential paths beginning at the agent's goal position
 */
void Planner::constructPath(std::vector<Position> *path, Position startPos, Position goalPos, Position intersectingPos, std::vector<std::vector<Position>> *prevPos, std::vector<std::vector<Position>> *nextPos) {
    /* Start from the intersecting position and work back to the start point */
    int x = intersectingPos.x;
    int y = intersectingPos.y;

    path->push_back(intersectingPos);
    while (prevPos->at(x).at(y).x != startPos.x || prevPos->at(x).at(y).y != startPos.y) {
        Position prev = prevPos->at(x).at(y);
        path->push_back(prev);
        x = prev.x;
        y = prev.y;
    }
    path->push_back(startPos);

    /* Get the first half of the path in the correct order */
    std::reverse(path->begin(), path->end());

    x = intersectingPos.x;
    y = intersectingPos.y;

    while (nextPos->at(x).at(y).x != goalPos.x || nextPos->at(x).at(y).y != goalPos.y) {
        Position next = nextPos->at(x).at(y);
        path->push_back(next);
        x = next.x;
        y = next.y;
    }
    path->push_back(goalPos);
}

/**
 * Finds the shortest path between the given positions. The given positions are NOT included in the path.
 *
 * @param startPos the agent's start position
 * @param goalPos the agent's target position
 * @return the shortest path
 */
std::vector<Position> Planner::getShortestPath(Position startPos, Position goalPos) {
    std::vector<Position> path = pathCache.get(startPos, goalPos);
    if (!path.empty()) {
        ROS_INFO("(getShortestPath) Path already exists, reusing it.");
        return path;
    }

    /* Previous position of a potential path for every position in the roadmap */
    std::vector<std::vector<Position>> prevPos(WIDTH, std::vector<Position>(HEIGHT, startPos));
    std::vector<std::vector<Position>> nextPos(WIDTH, std::vector<Position>(HEIGHT, goalPos));

    Position intersectingPos;
    if (planPath(startPos, goalPos, &prevPos, &nextPos, &intersectingPos) == false) {
        ROS_WARN("(getShortestPath) No path between points.");

        path.push_back(startPos);
        return path;
    }

    /* Positions in the shortest path */
    constructPath(&path, startPos, goalPos, intersectingPos, &prevPos, &nextPos);

    std::cout << "Shortest path: ";
    for (int i = 0; i < path.size(); i++) {
        Position p = path.at(i);
        std::cout << "(" << (int)p.x << "," << (int)p.y << "),";
        roadmap.set(p.x, p.y, i * COST_BETWEEN_COORDS);
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
