#ifndef COMMON_H
#define COMMON_H

/* Grid width */
#define WIDTH 10

/* Grid height */
#define HEIGHT 10

/* The topic for agent nodes to publish their position */
#define AGENT_FEEDBACK_TOPIC "agent_feedback"

/* The service for agent nodes to request a path from the planner node */
#define GET_PLAN_SERVICE "get_plan"

/* Topic queue size */
#define QUEUE_SIZE 1000

#endif
