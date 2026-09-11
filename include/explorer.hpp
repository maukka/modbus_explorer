#pragma once

#include <vector>

#include "maze_map.hpp"
#include "modbus_client.hpp"

/**
 * Class to implement the functionality to explore the given maze.
 */
class Explorer {
public:
    explicit Explorer(ModbusClient& client);

    void run();

private:
    void collectKeys();
    void unlockHatch(const std::vector<uint32_t>& codes);
    void findAndReachHatch();

    void revealNeighbours();                 // map-query unknown tiles adjacent to pos_
    bool hatchVisibleNearby(Point& outHatch); // check known neighbours for the hatch
    void applyMove(ModbusClient::Direction dir);
    void walkTo(Point target);

    ModbusClient& client_;
    MazeMap map_;
    Point pos_{0, 0}; // robot's position in our own coordinate frame, origin at
                       // wherever the robot happened to be when phase 2 started
};
