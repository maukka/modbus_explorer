#include <iostream>
#include <string>

#include "explorer.hpp"
#include "modbus_client.hpp"

int main(int argc, char** argv) {
    std::string host = argc > 1 ? argv[1] : "127.0.0.1";
    int port = argc > 2 ? std::stoi(argv[2]) : 5020;

    ModbusClient client(host, port);
    try {
        client.connect();
        Explorer explorer(client);
        explorer.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
