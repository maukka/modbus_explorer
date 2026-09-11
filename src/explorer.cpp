#include "explorer.hpp"

#include <iostream>
#include <numeric>
#include <stdexcept>

/**
 * Explicit constuctor to make sure no implicit type conversion is made.
 */
Explorer::Explorer(ModbusClient& client) : client_(client) {}

/**
 * Runs the full exploration sequence: collect keys, unlock the hatch, and explore until the hatch is reached.
 */
void Explorer::applyMove(ModbusClient::Direction dir) {
    client_.move(dir);
    switch (dir) {
        case ModbusClient::NORTH: pos_.y -= 1; break;
        case ModbusClient::EAST:  pos_.x += 1; break;
        case ModbusClient::SOUTH: pos_.y += 1; break;
        case ModbusClient::WEST:  pos_.x -= 1; break;
    }
}

/**
 * Collects all four keys by navigating to each one using the modbus server's navigation registers, 
 * reading their RFID codes, and then unlocking the hatch with the XOR of those codes.
 * @throws std::runtime_error if the modbus read or write operations fail, or if
 */
void Explorer::collectKeys() {
    std::vector<uint32_t> codes;
    for (int key = 0; key < 4; ++key) {
        std::cout << "Navigating to key " << (key + 1) << "...\n";
        uint16_t nav;
        uint16_t prevNav = 0;
        int sameCount = 0;
        uint16_t counterBefore = client_.readCounter();
        while ((nav = client_.readNav(key)) != 0) {
            auto dir = static_cast<ModbusClient::Direction>(nav);

            if (client_.readSurround(dir)) {
                std::cerr << "WARNING: nav says move " << nav
                          << " but that direction is a WALL "
                          << "(server-side bug or stuck position)\n";
            }

            sameCount = (nav == prevNav) ? sameCount + 1 : 0;
            prevNav = nav;
            if (sameCount == 60) {
                uint16_t counterNow = client_.readCounter();
                std::cerr << "Stuck: direction " << nav
                          << " repeated 60+ times without changing. "
                             "COUNTER before=" << counterBefore
                          << " now=" << counterNow
                          << " (changed=" << (counterNow != counterBefore) << ")\n";
                break;
            }

            applyMove(dir);
        }
        uint32_t code = client_.readKeyCode();
        std::cout << "  key " << (key + 1) << " RFID code: 0x" << std::hex
                  << code << std::dec << "\n";
        codes.push_back(code);
    }
    unlockHatch(codes);
}

/**
 * Unlocks the hatch by computing the XOR of the collected key codes and writing it to the modbus server.
 * @param codes The vector of 4 key RFID codes collected from the maze.
 * @throws std::runtime_error if the modbus write operation fails or if the hatch code is rejected by the server.
 */
void Explorer::unlockHatch(const std::vector<uint32_t>& codes) {
    uint32_t xorCode = std::accumulate(
        codes.begin(), codes.end(), static_cast<uint32_t>(0),
        [](uint32_t a, uint32_t b) { return a ^ b; });

    std::cout << "All 4 keys found, writing XOR code 0x" << std::hex
               << xorCode << std::dec << " to the hatch register...\n";
    client_.writeXorCode(xorCode);

    uint16_t state = client_.readState();
    if (state != ModbusClient::STATE_RFID_OK) {
        throw std::runtime_error("Hatch code rejected (state=" +
                                  std::to_string(state) + ")");
    }
    std::cout << "Hatch code accepted - map queries are now usable.\n";
}

/**
 * Reveals the tiles adjacent to the robot's current position by querying the modbus server for any unknown tiles.
 */
void Explorer::revealNeighbours() {
    for (const auto& n : MazeMap::neighbours(pos_)) {
        if (map_.get(n).has_value()) continue;
        int dx = n.x - pos_.x;
        int dy = n.y - pos_.y;
        map_.set(n, client_.queryMap(dx, dy));
    }
}

/**
 * Checks if the given point is hatch
 * @param Point containing x-y coordinate
 */
bool Explorer::hatchVisibleNearby(Point& outHatch) {
    for (const auto& n : MazeMap::neighbours(pos_)) {
        if (map_.get(n) == ModbusClient::TILE_HATCH) {
            outHatch = n;
            return true;
        }
    }
    return false;
}

/**
 * Moves to given point in maze
 * @param Point containg the x-y coordinate where we want to move
 */
void Explorer::walkTo(Point target) {
    for (auto dir : map_.pathTo(pos_, target)) {
        applyMove(dir);
    }
}

/**
 * Find and reach the hatch.
 */
void Explorer::findAndReachHatch() {
    map_.set(pos_, ModbusClient::TILE_EMPTY);

    while (true) {
        revealNeighbours();

        Point hatch;
        if (hatchVisibleNearby(hatch)) {
            walkTo(hatch);
            std::cout << "Reached the hatch.\n";
            return;
        }

        // Never brute-force scan the whole maze: always head for the
        // nearest still-unexplored boundary of what we already know.
        auto frontier = map_.nearestFrontier(pos_);
        if (!frontier) {
            throw std::runtime_error(
                "Explored every reachable tile but never found the hatch");
        }

        // Walk one step at a time towards the frontier, re-checking for the
        // hatch after every step in case it becomes visible along the way.
        for (auto dir : map_.pathTo(pos_, *frontier)) {
            applyMove(dir);
            revealNeighbours();
            if (hatchVisibleNearby(hatch)) {
                walkTo(hatch);
                std::cout << "Reached the hatch.\n";
                return;
            }
        }
    }
}

/**
 * Starts to explore the maze. Collects the keys, finds the hatches
 */
void Explorer::run() {
    uint32_t name = client_.readName();
    char c0 = static_cast<char>((name >> 24) & 0xFF);
    char c1 = static_cast<char>((name >> 16) & 0xFF);
    char c2 = static_cast<char>((name >> 8) & 0xFF);
    char c3 = static_cast<char>(name & 0xFF);
    std::cout << "Connected to " << c0 << c1 << c2 << c3 << "\n";

    collectKeys();
    findAndReachHatch();

    uint16_t state = client_.readState();
    if (state == ModbusClient::STATE_DONE) {
        std::cout << "IR-JA made it out of the sewers. Done!\n";
    } else {
        std::cout << "Reached what looked like the hatch, but final state is "
                  << state << " (expected " << ModbusClient::STATE_DONE
                  << ")\n";
    }
}
