#pragma once

#include <cstdint>
#include <string>

#include <modbus/modbus.h>

/**
 * A client for communicating with a modbus server that controls a robot in a maze.
 * Provides methods to move the robot, read its state, query the map, and interact with keys and the hatch.
 * Declared to be final	so that cannot be inherited from, ensuring that the interface remains consistent and preventing unintended modifications.
 * Also making sure that the destructor is virtual is not necessary since this class is not intended to be used as a base class.
 */
class ModbusClient final{
public:
    ModbusClient(std::string host, int port);
    ~ModbusClient();

    ModbusClient(const ModbusClient&) = delete;
    ModbusClient& operator=(const ModbusClient&) = delete;

    void connect();
    void disconnect();

    // --- Holding registers (FC03 read / FC06,FC16 write) ---
    static constexpr int REG_MOVE     = 0;  
    static constexpr int REG_NAME_HI  = 3;  
    static constexpr int REG_NAME_LO  = 4;
    static constexpr int REG_CODE_HI  = 5; 
    static constexpr int REG_CODE_LO  = 6;
    static constexpr int REG_QUERY_X  = 7;  
    static constexpr int REG_QUERY_Y  = 8;  

    // --- Input registers (FC04 read only) ---
    static constexpr int REG_NORTH      = 10;
    static constexpr int REG_EAST       = 11;
    static constexpr int REG_SOUTH      = 12;
    static constexpr int REG_WEST       = 13;
    static constexpr int REG_KEY_HI     = 14; 
    static constexpr int REG_KEY_LO     = 15;
    static constexpr int REG_MAP_RESULT = 17;
    static constexpr int REG_COUNTER    = 18;
    static constexpr int REG_STATE      = 19;

    enum Direction : uint16_t { NORTH = 1, EAST = 2, SOUTH = 3, WEST = 4 };

    enum State : uint16_t {
        STATE_SEARCHING = 0,
        STATE_RFID_OK   = 12,
        STATE_DONE      = 42,
    };

    enum Tile : uint16_t {
        TILE_EMPTY = 0,
        TILE_WALL  = 1,
        TILE_KEY   = 2,
        TILE_HATCH = 3,
    };

    void move(Direction dir);

    uint16_t readNav(int keyIndex);
    uint32_t readKeyCode();
    uint16_t readState();
    uint16_t readCounter();
    uint32_t readName();
    bool     readSurround(Direction dir);

    void writeXorCode(uint32_t code);
    Tile queryMap(int dx, int dy);

private:
    uint16_t readInput16(int addr);
    uint32_t readInput32(int addrHi);
    uint32_t readHolding32(int addrHi);

    std::string host_;
    int port_;
    modbus_t* ctx_ = nullptr;
};
