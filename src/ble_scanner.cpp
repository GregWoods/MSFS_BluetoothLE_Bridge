#include <functional>
#include <iomanip>
#include <iostream>
#include <string>

#include "simpleble/SimpleBLE.h"

// Shared session runner implemented in src/ble_session.cpp
int ble_run_session(const std::string& device_identifier,
                    const std::string& characteristic_uuid,
                    int scan_timeout_sec,
                    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet);

static void print_hex_bytes(const SimpleBLE::ByteArray& data, const std::string& devTag) {
    std::cout << "[" << devTag << "] (" << data.size() << " bytes): ";
    for (unsigned char c : data) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(c) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;
}

int main() {
    constexpr const char* DEVICE_IDENTIFIER = "SHB1000";
    constexpr const char* CHARACTERISTIC_UUID = "f62a9f56-f29e-48a8-a317-47ee37a58999";
    constexpr int SCAN_TIMEOUT_SEC = 20;

    auto on_packet = [](const SimpleBLE::ByteArray& bytes, const std::string& devTag) {
        // Scanner-only behavior: just print the bytes
        print_hex_bytes(bytes, devTag);
    };

    return ble_run_session(DEVICE_IDENTIFIER, CHARACTERISTIC_UUID, SCAN_TIMEOUT_SEC, on_packet);
}