#include "modbus_client.hpp"

#include <cerrno>
#include <stdexcept>

ModbusClient::ModbusClient(std::string host, int port)
    : host_(std::move(host)), port_(port) {}

/**
 * Destructor: disconnects from the modbus server and releases resources. Safe to call even if not connected.
 */
ModbusClient::~ModbusClient() { disconnect(); }

/**
 * Connects to modbus server and sets the slave ID to 1 (the only one used by the maze server).
 */
void ModbusClient::connect() {
    ctx_ = modbus_new_tcp(host_.c_str(), port_);
    if (!ctx_) {
        throw std::runtime_error("modbus_new_tcp failed");
    }
    if (modbus_connect(ctx_) == -1) {
        std::string err = modbus_strerror(errno);
        modbus_free(ctx_);
        ctx_ = nullptr;
        throw std::runtime_error("modbus_connect failed: " + err);
    }

	modbus_set_slave(ctx_, 1);
	#ifdef DEBUG
		modbus_set_debug(ctx_, TRUE);
	#endif	
}

/**
 * Disconnect from modbus server and release the resources. Safe to call even if not connected.
 */
void ModbusClient::disconnect() {
    if (ctx_) {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
}

/**
 * Moves the robot in the specified direction.
 * @param dir The direction to move (NORTH, EAST, SOUTH, WEST).
 * @throws std::runtime_error if the modbus write operation fails.	
 */
void ModbusClient::move(Direction dir) {
    if (modbus_write_register(ctx_, REG_MOVE, static_cast<uint16_t>(dir)) == -1) {
        throw std::runtime_error(std::string("move failed: ") + modbus_strerror(errno));
    }
}

/**
 * Read	 a 16-bit input register from the modbus server.
 * @param addr The address of the input register to read.
 * @return The value of the input register.
 * @throws std::runtime_error if the modbus read operation fails.
 */	
uint16_t ModbusClient::readInput16(int addr) {
    uint16_t v = 0;
    if (modbus_read_input_registers(ctx_, addr, 1, &v) == -1) {
        throw std::runtime_error(std::string("readInput16 failed: ") + modbus_strerror(errno));
    }
    return v;
}

/**
 * Read a 32-bit input register from the modbus server (two consecutive 16-bit registers).
 * @param addrHigh The address of the high 16 bits of the input register to read.
 * @return The value of the 32-bit input register.
 * @throws std::runtime_error if the modbus read operation fails.
 */
uint32_t ModbusClient::readInput32(int addrHigh) {
    uint16_t buf[2] = {0, 0};
    if (modbus_read_input_registers(ctx_, addrHigh, 2, buf) == -1) {
        throw std::runtime_error(std::string("readInput32 failed: ") + modbus_strerror(errno));
    }
    return (static_cast<uint32_t>(buf[0]) << 16) | buf[1];
}

/**
 * Read a 32-bit holding register from the modbus server (two consecutive 16-bit registers).
 * @param addrHigh The address of the high 16 bits of the holding register to read.
 * @return The value of the 32-bit holding register.
 * @throws std::runtime_error if the modbus read operation fails.		
 */
uint32_t ModbusClient::readHolding32(int addrHigh) {
    uint16_t buf[2] = {0, 0};
    if (modbus_read_registers(ctx_, addrHigh, 2, buf) == -1) {
        throw std::runtime_error(std::string("readHolding32 failed: ") + modbus_strerror(errno));
    }
    return (static_cast<uint32_t>(buf[0]) << 16) | buf[1];
}

/**
 * Read the navigation direction for a specific key from the modbus server.
 * @param keyIndex The index of the key (0 to 3) for 4 keys to find.
 * @return The direction towards the key (0 if standing on it).
 * @throws std::runtime_error if the modbus read operation fails.
	 */
uint16_t ModbusClient::readNav(int keyIndex) {
    return readInput16(REG_NAV_BASE + keyIndex);
}

/**
 * Public methods to access the modbus registers.
 */
uint32_t ModbusClient::readKeyCode() { return readInput32(REG_KEY_HI); }

uint16_t ModbusClient::readState() { return readInput16(REG_STATE); }

uint16_t ModbusClient::readCounter() { return readInput16(REG_COUNTER); }

uint32_t ModbusClient::readName() { return readHolding32(REG_NAME_HI); }

/**
 * Reads the surrounding tile in the specified direction from the modbus server.
 * @param dir The direction to check (NORTH, EAST, SOUTH, WEST).
 * @return true if the tile in that direction is a wall, false otherwise.
 * @throws std::runtime_error if the modbus read operation fails or if the direction is
 */
bool ModbusClient::readSurround(Direction dir) {
    int addr;
    switch (dir) {
        case NORTH: addr = REG_NORTH; break;
        case EAST:  addr = REG_EAST;  break;
        case SOUTH: addr = REG_SOUTH; break;
        case WEST:  addr = REG_WEST;  break;
        default: throw std::runtime_error("invalid direction");
    }
    return readInput16(addr) == TILE_WALL;
}

/**
 * Writes the XOR code to the modbus server to unlock the hatch.
 * @param code The 32-bit XOR code to write.
 * @throws std::runtime_error if the modbus write operation fails.
 */
void ModbusClient::writeXorCode(uint32_t code) {
    uint16_t buf[2];
    buf[0] = static_cast<uint16_t>(code >> 16);
    buf[1] = static_cast<uint16_t>(code & 0xFFFF);
    // Both registers must land in a single write so the server never reads
    // a half-updated value (see README "Modbus note").
    if (modbus_write_registers(ctx_, REG_CODE_HI, 2, buf) == -1) {
        throw std::runtime_error(std::string("writeXorCode failed: ") + modbus_strerror(errno));
    }
}

/**
 * Queries the map for the tile at a relative position (dx, dy) from the robot's current position.
 * @param dx The relative x-coordinate (positive = east, negative = west).
 * @param dy The relative y-coordinate (positive = south, negative = north).
 * @return The tile type at the queried position (TILE_EMPTY, TILE_WALL, TILE_KEY, TILE_HATCH).
 * @throws std::runtime_error if the modbus write or read operation fails.	
 */
ModbusClient::Tile ModbusClient::queryMap(int dx, int dy) {
    uint16_t buf[2];
    buf[0] = static_cast<uint16_t>(static_cast<int16_t>(dx)); // two's complement
    buf[1] = static_cast<uint16_t>(static_cast<int16_t>(dy));
    if (modbus_write_registers(ctx_, REG_QUERY_X, 2, buf) == -1) {
        throw std::runtime_error(std::string("queryMap write failed: ") + modbus_strerror(errno));
    }
    return static_cast<Tile>(readInput16(REG_MAP_RESULT));
}
