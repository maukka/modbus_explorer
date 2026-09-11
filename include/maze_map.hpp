#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "modbus_client.hpp"

struct Point {
    int x = 0;
    int y = 0;
    bool operator==(const Point& o) const { return x == o.x && y == o.y; }
};

struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        std::size_t h1 = std::hash<int>()(p.x);
        std::size_t h2 = std::hash<int>()(p.y);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

// Incrementally-built model of the maze, learned tile by tile as the robot
// moves and issues map queries. Unknown tiles simply aren't in the map yet
// — we never assume anything about territory we haven't observed.
class MazeMap {
public:
    void set(Point p, ModbusClient::Tile tile);
    std::optional<ModbusClient::Tile> get(Point p) const;
    bool isKnownFree(Point p) const;
    bool isFrontier(Point p) const; // known & free, with at least one unknown neighbour

    // Shortest path (as a direction sequence) between two known-free
    // points, staying entirely within known-free tiles. Empty if `from`
    // already equals `to`, and also empty (check via caller logic) if
    // genuinely unreachable through known tiles.
    std::vector<ModbusClient::Direction> pathTo(Point from, Point to) const;

    // Closest known-free cell (BFS distance) adjacent to unexplored
    // territory, reachable from `from` without leaving known-free tiles.
    // This is how exploration proceeds without ever brute-force scanning
    // the whole (potentially huge) maze.
    std::optional<Point> nearestFrontier(Point from) const;

    static std::vector<Point> neighbours(Point p);
    static ModbusClient::Direction directionTo(Point from, Point to);

private:
    std::unordered_map<Point, ModbusClient::Tile, PointHash> tiles_;
};
