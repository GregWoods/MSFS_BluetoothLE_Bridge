#include <array>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>

#include "simpleble/SimpleBLE.h"
#include "WASMIF.h"

constexpr const char* SIMIONIC_G1000_IDENTIFIER = "SHB1000";
constexpr const char* BLE_CHARACTERISTIC_UUID = "f62a9f56-f29e-48a8-a317-47ee37a58999";
constexpr int BLUETOOTH_SCANNING_TIMEOUT_SEC = 20;

static_assert(sizeof(void*) == 8, "Must build 64-bit (x64).");

static WASMIF* wasmPtr = nullptr;
static std::array<char, MAX_CALC_CODE_SIZE> ccode{};

class WASMIFGuard {
public:
    explicit WASMIFGuard(WASMIF* p) : p_(p) {}
    ~WASMIFGuard() { if (p_) p_->end(); }
    WASMIFGuard(const WASMIFGuard&) = delete;
    WASMIFGuard& operator=(const WASMIFGuard&) = delete;
private:
    WASMIF* p_;
};

static void print_hex_bytes(const SimpleBLE::ByteArray& data, const std::string& devTag) {
    std::cout << "[" << devTag << "] (" << data.size() << " bytes): ";
    for (unsigned char c : data) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(c) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;
}

// Shared session runner implemented in src/ble_session.cpp
int ble_run_session(const std::string& device_identifier,
                    const std::string& characteristic_uuid,
                    int scan_timeout_sec,
                    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet);

int main() {
    // SimConnect / WASM init
    wasmPtr = WASMIF::GetInstance();
    WASMIFGuard guard(wasmPtr);
    wasmPtr->setSimConfigConnection(SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
    wasmPtr->start();

    // Prepare calculator code (replayed on each incoming BLE packet)
    const char* calcCode = "(>H:AS1000_PFD_SOFTKEYS_2)";
    strncpy_s(ccode.data(), ccode.size(), calcCode, _TRUNCATE);

    auto on_packet = [](const SimpleBLE::ByteArray& bytes, const std::string& devTag) {
        if (wasmPtr) wasmPtr->executeCalclatorCode(ccode.data());
        print_hex_bytes(bytes, devTag);
    };

    return ble_run_session(
        SIMIONIC_G1000_IDENTIFIER,
        BLE_CHARACTERISTIC_UUID,
        BLUETOOTH_SCANNING_TIMEOUT_SEC,
        on_packet
    );
}