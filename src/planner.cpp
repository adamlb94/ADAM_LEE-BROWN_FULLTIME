#include "common.h"
#include <unordered_map>

const std::string NODE_NAME = "planner";

/**
 * Planning node's representation of an agent.
 */
class Agent {
private:
    std::string serial_id;
    Position currentPos;
    Position goalPos;

public:
    Agent(std::string id, Position pos) {
        serial_id = id;
        setCurrentPos(pos);
    }

    Position getCurrentPos() {
        return currentPos;
    }

    Position getGoalPos() {
        return goalPos;
    }

    std::string getId() {
        return serial_id;
    }

    void setCurrentPos(Position pos) {
        currentPos = pos;
    }

    void setGoalPos(Position pos) {
        goalPos = pos;
    }

    std::string description() {
        std::ostringstream oss;
        oss << serial_id << " "
            << "Current pos: (" << currentPos.x << "," << currentPos.y << "," <<  currentPos.theta << ") "
            << "Goal pos: (" << goalPos.x << "," << goalPos.y << "," <<  goalPos.theta << ")";
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
        std::string id = (std::string)msg->serial_id;
        Position pos;
        pos.x = (int)msg->x;
        pos.y = (int)msg->y;
        pos.theta = (int)msg->theta;

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
        std::string id = (std::string)req.serial_id;
        auto it = agents.find(id);

        if (it == agents.end()) {
            ROS_WARN("(getPlanCallback) agent not found!");
            res.x = -1;
            res.y = -1;
            res.theta = -1;
            return false;
        }

        Position currentPos = it->second.getCurrentPos();
        // Position goalPos = it->second.getGoalPos();
        res.x = 1;//currentPos.x + goalPos.x;
        res.y = 2;//currentPos.y + goalPos.y;
        res.theta = 3;//currentPos.theta + goalPos.theta;

        ROS_INFO("(getPlanCallback) Created plan for agent serial_id=%s: x=%d, y=%d, theta=%d", ((std::string)req.serial_id).c_str(), (int)res.x, (int)res.y, (int)res.theta);

        return true;
    }

    /**
     * Callback for the update_goal service.
     */
    bool updateGoalCallback(multi_agent_planning::UpdateGoal::Request  &req,
                            multi_agent_planning::UpdateGoal::Response &res) {
        std::string id = (std::string)req.serial_id;
        Position pos;
        pos.x = (int)req.x;
        pos.y = (int)req.y;
        pos.theta = (int)req.theta;

        auto it = agents.find(id);
        if (it == agents.end()) {
            // TODO: handle if agent does not exist in map
        } else {
            it->second.setGoalPos(pos);
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