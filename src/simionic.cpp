#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>
#include <thread>
#include <chrono>
#include <array>
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

static std::optional<SimpleBLE::Adapter> get_first_adapter() {
    try {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) return std::nullopt;
        return adapters.front();
    } catch (const std::exception& e) {
        std::cerr << "Adapter enumeration failed: " << e.what() << std::endl;
        return std::nullopt;
    }
}

static std::string to_lower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return v;
}

static void print_hex_bytes(const SimpleBLE::ByteArray& data, const std::string& devTag) {
    std::cout << "[" << devTag << "] (" << data.size() << " bytes): ";
    for (unsigned char c : data) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(c) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;
}

struct ConnectedDevice {
    SimpleBLE::Peripheral peripheral;
    SimpleBLE::BluetoothUUID service_uuid;
    SimpleBLE::BluetoothUUID characteristic_uuid;
    bool used_indicate = false;
};

static bool find_characteristic(SimpleBLE::Peripheral& p,
                                const std::string& desired_lower,
                                SimpleBLE::BluetoothUUID& out_service,
                                SimpleBLE::BluetoothUUID& out_char) {
    try {
        for (auto& service : p.services()) {
            for (auto& chr : service.characteristics()) {
                if (to_lower(chr.uuid()) == desired_lower) {
                    out_service = service.uuid();
                    out_char    = chr.uuid();
                    return true;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Service discovery failed on " << p.identifier()
                  << ": " << e.what() << std::endl;
    }
    return false;
}

int main() {
    // SimConnect / WASM init
    wasmPtr = WASMIF::GetInstance();
    WASMIFGuard guard(wasmPtr);
    wasmPtr->setSimConfigConnection(SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
    wasmPtr->start();

    // Prepare calculator code (replayed on each incoming BLE packet)
    const char* calcCode = "(>H:AS1000_PFD_SOFTKEYS_2)";
    strncpy_s(ccode.data(), ccode.size(), calcCode, _TRUNCATE);

    const std::string desired_char_lower = to_lower(BLE_CHARACTERISTIC_UUID);

    auto adapter_opt = get_first_adapter();
    if (!adapter_opt) {
        std::cerr << "No Bluetooth adapter found." << std::endl;
        return EXIT_FAILURE;
    }
    auto adapter = *adapter_opt;

    std::vector<SimpleBLE::Peripheral> scanned;
    scanned.reserve(32);
    std::unordered_set<std::string> seen_addresses;

    adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral p) {
        if (!p.is_connectable()) return;
        auto addr = p.address();
        if (!addr.empty() && seen_addresses.insert(addr).second) {
            std::cout << "Found device: " << p.identifier()
                      << " [" << addr << "]" << std::endl;
            scanned.push_back(p);
        }
    });
    adapter.set_callback_on_scan_start([](){ std::cout << "Scan started." << std::endl; });
    adapter.set_callback_on_scan_stop([](){ std::cout << "Scan stopped." << std::endl; });

    adapter.scan_for(BLUETOOTH_SCANNING_TIMEOUT_SEC * 1000);

    if (scanned.empty()) {
        std::cerr << "No connectable peripherals discovered." << std::endl;
        return EXIT_FAILURE;
    }

    // Filter SHB1000 devices (all of them)
    std::vector<SimpleBLE::Peripheral> targets;
    for (auto& p : scanned) {
        if (p.identifier() == SIMIONIC_G1000_IDENTIFIER) targets.push_back(p);
    }

    if (targets.empty()) {
        std::cerr << "No SHB1000 devices found." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Connecting to " << targets.size() << " SHB1000 device(s)..." << std::endl;

    std::vector<ConnectedDevice> connected;
    connected.reserve(targets.size());

    for (auto& p : targets) {
        std::cout << "-> Connecting: " << p.identifier()
                  << " [" << p.address() << "]" << std::endl;
        try {
            p.connect();
        } catch (const std::exception& e) {
            std::cerr << "   Connection failed: " << e.what() << " (skipping)" << std::endl;
            continue;
        }

        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID char_uuid;
        if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
            std::cerr << "   Target characteristic not found on "
                      << p.address() << " (skipping)" << std::endl;
            try { p.disconnect(); } catch (...) {}
            continue;
        }

        bool subscribed = false;
        bool used_indicate = false;

        try {
            bool can_indicate = false;
            bool can_notify   = false;

            // Re-scan service characteristics to inspect properties
            for (auto& service : p.services()) {
                if (service.uuid() != service_uuid) continue;
                for (auto& chr : service.characteristics()) {
                    if (chr.uuid() != char_uuid) continue;
                    can_indicate = chr.can_indicate();
                    can_notify   = chr.can_notify();
                    break;
                }
            }

            auto devTag = p.address();
            if (can_indicate) {
                p.indicate(service_uuid, char_uuid,
                           [devTag](SimpleBLE::ByteArray bytes) {
                               if (wasmPtr) wasmPtr->executeCalclatorCode(ccode.data());
                               print_hex_bytes(bytes, devTag);
                           });
                subscribed = true;
                used_indicate = true;
            } else if (can_notify) {
                p.notify(service_uuid, char_uuid,
                         [devTag](SimpleBLE::ByteArray bytes) {
                             if (wasmPtr) wasmPtr->executeCalclatorCode(ccode.data());
                             print_hex_bytes(bytes, devTag);
                         });
                subscribed = true;
            } else {
                std::cerr << "   Char supports neither indicate nor notify on "
                          << devTag << " (skipping)" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "   Subscription failed: " << e.what() << " (disconnecting)" << std::endl;
            try { p.disconnect(); } catch (...) {}
            subscribed = false;
        }

        if (subscribed) {
            std::cout << "   " << (used_indicate ? "Indication" : "Notification")
                      << " active on " << p.address() << std::endl;
            connected.push_back({p, service_uuid, char_uuid, used_indicate});
        }
    }

    if (connected.empty()) {
        std::cerr << "No devices successfully subscribed." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Listening on " << connected.size()
              << " device(s). Press Enter to stop..." << std::endl;
    if (std::cin.peek() == '\n') std::cin.get();
    std::string line;
    std::getline(std::cin, line);

    // Unsubscribe & disconnect all
    for (auto& cd : connected) {
        try {
            cd.peripheral.unsubscribe(cd.service_uuid, cd.characteristic_uuid);
        } catch (...) {}
        try {
            cd.peripheral.disconnect();
        } catch (...) {}
    }

    std::cout << "Disconnected all. Exiting." << std::endl;
    return EXIT_SUCCESS;
}