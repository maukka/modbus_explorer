#include "maze_map.hpp"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <unordered_set>

using Tile = ModbusClient::Tile;
using Direction = ModbusClient::Direction;

void MazeMap::set(Point p, Tile tile) { tiles_[p] = tile; }

std::optional<Tile> MazeMap::get(Point p) const {
    auto it = tiles_.find(p);
    if (it == tiles_.end()) return std::nullopt;
    return it->second;
}

bool MazeMap::isKnownFree(Point p) const {
    auto t = get(p);
    return t.has_value() && *t != Tile::TILE_WALL;
}

std::vector<Point> MazeMap::neighbours(Point p) {
    return {{p.x, p.y - 1}, {p.x + 1, p.y}, {p.x, p.y + 1}, {p.x - 1, p.y}};
}

Direction MazeMap::directionTo(Point from, Point to) {
    if (to.x == from.x && to.y == from.y - 1) return Direction::NORTH;
    if (to.x == from.x + 1 && to.y == from.y) return Direction::EAST;
    if (to.x == from.x && to.y == from.y + 1) return Direction::SOUTH;
    if (to.x == from.x - 1 && to.y == from.y) return Direction::WEST;
    throw std::runtime_error("MazeMap::directionTo: points are not adjacent");
}

bool MazeMap::isFrontier(Point p) const {
    if (!isKnownFree(p)) return false;
    for (const auto& n : neighbours(p)) {
        if (!get(n).has_value()) return true;
    }
    return false;
}

std::vector<Direction> MazeMap::pathTo(Point from, Point to) const {
    if (from == to) return {};

    std::unordered_map<Point, Point, PointHash> cameFrom;
    std::unordered_set<Point, PointHash> visited;
    std::queue<Point> q;
    q.push(from);
    visited.insert(from);
    bool found = false;

    while (!q.empty() && !found) {
        Point cur = q.front();
        q.pop();
        for (const auto& n : neighbours(cur)) {
            if (visited.count(n)) continue;
            if (!isKnownFree(n)) continue;
            visited.insert(n);
            cameFrom[n] = cur;
            if (n == to) {
                found = true;
                break;
            }
            q.push(n);
        }
    }
    if (!found) return {};

    std::vector<Point> reversed;
    Point cur = to;
    while (!(cur == from)) {
        reversed.push_back(cur);
        cur = cameFrom[cur];
    }
    std::reverse(reversed.begin(), reversed.end());

    std::vector<Direction> dirs;
    dirs.reserve(reversed.size());
    Point prev = from;
    for (const auto& p : reversed) {
        dirs.push_back(directionTo(prev, p));
        prev = p;
    }
    return dirs;
}

std::optional<Point> MazeMap::nearestFrontier(Point from) const {
    if (isFrontier(from)) return from;

    std::unordered_set<Point, PointHash> visited;
    std::queue<Point> q;
    q.push(from);
    visited.insert(from);

    while (!q.empty()) {
        Point cur = q.front();
        q.pop();
        for (const auto& n : neighbours(cur)) {
            if (visited.count(n)) continue;
            if (!isKnownFree(n)) continue;
            visited.insert(n);
            if (isFrontier(n)) return n;
            q.push(n);
        }
    }
    return std::nullopt;
}
