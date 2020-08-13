#include "common.h"
#include <unordered_map>

const std::string NODE_NAME = "planner";

/**
 * Planning node's representation of an agent.
 */
class Agent {
private:
    std::string id;
    multi_agent_planning::Position currentPos;
    multi_agent_planning::Position goalPos;
    /* Planned path, not including the current and goal position */
    std::vector<multi_agent_planning::Position> path;

public:
    Agent(std::string id, multi_agent_planning::Position pos) {
        this->id = id;
        setCurrentPos(pos);
    }

    multi_agent_planning::Position getCurrentPos() {
        return currentPos;
    }

    multi_agent_planning::Position getGoalPos() {
        return goalPos;
    }

    std::string getId() {
        return id;
    }

    std::vector<multi_agent_planning::Position> getPath() {
        return path;
    }

    void setCurrentPos(multi_agent_planning::Position pos) {
        currentPos = pos;
    }

    void setGoalPos(multi_agent_planning::Position pos) {
        goalPos = pos;
    }

    void setPath(std::vector<multi_agent_planning::Position> path) {
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
 * Planning node that plans paths for agents.
 */
class Planner {
private:
    std::unique_ptr<ros::NodeHandle> nodeHandle;
    ros::Subscriber agentFeedbackSubscriber;
    ros::ServiceServer getPlanServer;
    ros::ServiceServer updateGoalServer;

    std::unordered_map<std::string, Agent> agents;

    /**
     * Callback for the agent_feedback topic.
     */
    void agentFeedbackCallback(const multi_agent_planning::AgentPos::ConstPtr& msg) {
        std::string id = (std::string)msg->id;
        multi_agent_planning::Position pos = (multi_agent_planning::Position)msg->position;

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

    /**
     * Callback for the get_plan service.
     */
    bool getPlanCallback(multi_agent_planning::GetPlan::Request  &req,
                         multi_agent_planning::GetPlan::Response &res) {
        std::string id = (std::string)req.id;
        auto it = agents.find(id);

        if (it == agents.end()) {
            ROS_WARN("(getPlanCallback) agent not found!");
            // TODO: Need to return message?
            return false;
        }

        std::vector<multi_agent_planning::Position> path;
        path.push_back(it->second.getCurrentPos());
        path.push_back(it->second.getGoalPos());
        it->second.setPath(path);

        res.path = path;

        ROS_INFO("(getPlanCallback) Created plan for agent id=%s", ((std::string)req.id).c_str());
        return true;
    }

    /**
     * Callback for the update_goal service.
     */
    bool updateGoalCallback(multi_agent_planning::UpdateGoal::Request  &req,
                            multi_agent_planning::UpdateGoal::Response &res) {
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