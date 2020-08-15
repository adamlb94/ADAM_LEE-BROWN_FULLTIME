#include "multi_agent_planning/Position.h"

/**
 * A 2D coordinate.
 */
struct Point {
    int x;
    int y;

    /**
     * Define '<' operator to allow Points to be used as map keys.
     */
    bool operator<(const Point p) const;
};

/**
 * Cache for planned paths.
 */
class PathCache {
private:
    /**
     * Returns the Point representation of the given Position.
     *
     * @param pos the Position
     * @return the Point representation
     */
    Point toPoint(multi_agent_planning::Position pos);

public:
    /**
     * Returns the shortest path of Positions between the given points, inclusive, if it exists. Returns empty path otherwise.
     *
     * @param startPos the start Position
     * @param endPos the end Position
     * @return the shortest path, if it exists
     */
    std::vector<multi_agent_planning::Position> get(multi_agent_planning::Position startPos, multi_agent_planning::Position endPos);

    /**
     * Add the given path to the path cache.
     *
     * @param path the path to add
     */
    void put(std::vector<multi_agent_planning::Position> path);

};