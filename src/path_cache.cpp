#include "path_cache.h"

using namespace multi_agent_planning;

/**
 * Define '<' operator to allow Points to be used as map keys.
 */
bool Point::operator<(const Point p) const {
    return x < p.x || (x == p.x && y < p.y);
}

/**
 * Returns the Point representation of the given Position.
 *
 * @param pos the Position
 * @return the Point representation
 */
Point PathCache::toPoint(Position pos) {
    Point p;
    p.x = pos.x;
    p.y = pos.y;
    return p;
}

/**
 * Returns the shortest path of Positions between the given points, inclusive, if it exists. Returns empty path otherwise.
 *
 * @param startPos the start Position
 * @param endPos the end Position
 * @return the shortest path, if it exists
 */
std::vector<Position> PathCache::get(Position startPos, Position endPos) {
    std::vector<Position> path;

    Point start = toPoint(startPos);
    auto startIt = paths.find(start);
    if (startIt != paths.end()) {
        Point end = toPoint(endPos);

        auto endIt = startIt->second.find(end);
        if (endIt != startIt->second.end()) {
            std::cout << "(Cache.get) Path exists" << std::endl;
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
void PathCache::put(std::vector<Position> path) {
    int pathSize = path.size();
    if (pathSize < 2) {
        std::cout << "(Cache.put) Given path is too short" << std::endl;
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
                std::cout << "(Cache.put) Updating path" << std::endl;
                endIt->second = path;
            }
        } else {
            /* Add end-path entry start point map */
            std::cout << "(Cache.put) Start point exists, adding new path" << std::endl;
            startIt->second.insert({end, path});
        }
    } else {
        /* New start point */
        std::map<Point, std::vector<Position>> endPathMap;
        endPathMap.insert({end, path});

        paths.insert({start, endPathMap});
        std::cout << "(Cache.put) Adding new path" << std::endl;
    }
}
