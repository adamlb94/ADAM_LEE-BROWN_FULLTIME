#ifndef PATH_CACHE_H
#define PATH_CACHE_H

#include "multi_agent_planning/Position.h"

/**
 * A 2D coordinate.
 */
struct Point {
    int x;
    int y;
    bool operator<(const Point p) const;
};

/**
 * Cache for planned paths.
 */
class PathCache {
private:
    /* Paths cache with structure map<startPoint, map<endPoint, path>> */
    std::map<Point, std::map<Point, std::vector<multi_agent_planning::Position>>> paths;

    Point toPoint(multi_agent_planning::Position pos);

public:
    std::vector<multi_agent_planning::Position> get(multi_agent_planning::Position startPos, multi_agent_planning::Position endPos);
    void put(std::vector<multi_agent_planning::Position> path);
};

#endif
