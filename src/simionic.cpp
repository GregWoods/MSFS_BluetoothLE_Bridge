#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>
#include "simpleble/SimpleBLE.h"

constexpr const char* SIMIONIC_G1000_IDENTIFIER      = "SHB1000";
constexpr const char* BLE_CHARACTERISTIC_UUID        = "f62a9f56-f29e-48a8-a317-47ee37a58999";
constexpr int        BLUETOOTH_SCANNING_TIMEOUT_SEC  = 20;

static std::optional<SimpleBLE::Adapter> get_first_adapter() {
    std::vector<SimpleBLE::Adapter> adapters;
    try {
        adapters = SimpleBLE::Adapter::get_adapters();
    }
    catch (const std::exception& e) {
        std::cerr << "Adapter enumeration failed: " << e.what() << std::endl;
        return std::nullopt;
    }
    if (adapters.empty()) return std::nullopt;
    return adapters.front();
}

static std::optional<size_t> get_user_input_int(const std::string& prompt, size_t max_index) {
    std::cout << prompt << " (0-" << max_index << "): ";
    std::string line;
    if (!std::getline(std::cin, line)) return std::nullopt;
    if (line.empty()) return std::nullopt;
    try {
        size_t pos = 0;
        unsigned long v = std::stoul(line, &pos, 10);
        if (pos != line.size() || v > max_index) return std::nullopt;
        return static_cast<size_t>(v);
    }
    catch (...) {
        return std::nullopt;
    }
}

static std::string to_lower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}

static void print_hex_bytes(const SimpleBLE::ByteArray& data) {
    std::cout << "Indication (" << data.size() << " bytes): ";
    for (unsigned char c : data) {
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                  << static_cast<int>(c) << " ";
    }
    std::cout << std::dec << std::nouppercase << std::endl;
}

static void on_receive_bytes(const SimpleBLE::ByteArray& bytes) {
    // TODO: Integrate SimConnect / FSUIPC handling here.
    print_hex_bytes(bytes);
}

int main() {
    const std::string desired_characteristic_uuid = to_lower(BLE_CHARACTERISTIC_UUID);

    auto adapter_optional = get_first_adapter();
    if (!adapter_optional.has_value()) {
        std::cerr << "No Bluetooth adapter found." << std::endl;
        return EXIT_FAILURE;
    }
    auto adapter = adapter_optional.value();

    std::vector<SimpleBLE::Peripheral> scanned_peripherals;
    scanned_peripherals.reserve(32);
    std::unordered_set<std::string> seen_addresses;

    adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral peripheral) {
        if (!peripheral.is_connectable()) return;
        std::string addr = peripheral.address();
        if (!addr.empty() && seen_addresses.insert(addr).second) {
            std::cout << "Found device: " << peripheral.identifier()
                      << " [" << addr << "]" << std::endl;
            scanned_peripherals.push_back(peripheral);
        }
    });
    adapter.set_callback_on_scan_start([]() { std::cout << "Scan started." << std::endl; });
    adapter.set_callback_on_scan_stop([]() { std::cout << "Scan stopped." << std::endl; });

    adapter.scan_for(BLUETOOTH_SCANNING_TIMEOUT_SEC * 1000);

    if (scanned_peripherals.empty()) {
        std::cerr << "No connectable peripherals discovered." << std::endl;
        return EXIT_FAILURE;
    }

    // Filter ONLY SHB1000 devices.
    std::vector<SimpleBLE::Peripheral> simionic_peripherals;
    for (auto& p : scanned_peripherals) {
        if (p.identifier() == SIMIONIC_G1000_IDENTIFIER) {
            simionic_peripherals.push_back(p);
        }
    }

    if (simionic_peripherals.empty()) {
        std::cerr << "No Simionic G1000 devices (identifier: "
                  << SIMIONIC_G1000_IDENTIFIER << ") found." << std::endl;
        return EXIT_FAILURE;
    }

    size_t chosen_index = 0;
    if (simionic_peripherals.size() == 1) {
        std::cout << "One SHB1000 device found. Auto-selecting it." << std::endl;
    } else {
        std::cout << "Simionic G1000 devices:" << std::endl;
        for (size_t i = 0; i < simionic_peripherals.size(); ++i) {
            std::cout << "[" << i << "] "
                      << simionic_peripherals[i].identifier()
                      << " [" << simionic_peripherals[i].address() << "]"
                      << std::endl;
        }
        auto selection = get_user_input_int("Select device index",
                                                simionic_peripherals.size() - 1);
        if (!selection.has_value()) {
            std::cerr << "Invalid selection." << std::endl;
            return EXIT_FAILURE;
        }
        chosen_index = selection.value();
    }

    auto peripheral = simionic_peripherals[chosen_index];
    std::cout << "Connecting to " << peripheral.identifier()
              << " [" << peripheral.address() << "]" << std::endl;

    try {
        peripheral.connect();
    } catch (const std::exception& e) {
        std::cerr << "Connection failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Retrieve services.
    auto services = peripheral.services();

    SimpleBLE::BluetoothUUID service_uuid;
    SimpleBLE::BluetoothUUID characteristic_uuid;
    bool characteristic_found = false;

    try {
        for (auto& service : services) {
            for (auto& characteristic : service.characteristics()) {
                if (to_lower(characteristic.uuid()) == desired_characteristic_uuid) {
                    service_uuid = service.uuid();
                    characteristic_uuid = characteristic.uuid();
                    characteristic_found = true;
                    break;
                }
            }
            if (characteristic_found) break;
        }
    } catch (const std::exception& e) {
        std::cerr << "Service discovery failed: " << e.what() << std::endl;
        peripheral.disconnect();
        return EXIT_FAILURE;
    }

    if (!characteristic_found) {
        std::cerr << "Characteristic " << BLE_CHARACTERISTIC_UUID
                  << " not found on selected device." << std::endl;
        peripheral.disconnect();
        return EXIT_FAILURE;
    }

    bool subscribed = false;
    bool used_indicate = false;

    try {
        bool can_indicate = false;
        bool can_notify = false;

        // Determine capability flags.
        for (auto& service : services) {
            if (service.uuid() != service_uuid) continue;
            for (auto& c : service.characteristics()) {
                if (c.uuid() != characteristic_uuid) continue;
                can_indicate = c.can_indicate();
                can_notify   = c.can_notify();
                break;
            }
        }

        if (can_indicate) {
            peripheral.indicate(service_uuid, characteristic_uuid,
                                [&](SimpleBLE::ByteArray bytes) { on_receive_bytes(bytes); });
            subscribed = true;
            used_indicate = true;
        } else if (can_notify) {
            peripheral.notify(service_uuid, characteristic_uuid,
                              [&](SimpleBLE::ByteArray bytes) { on_receive_bytes(bytes); });
            subscribed = true;
        } else {
            std::cerr << "Characteristic supports neither indicate nor notify." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Subscription failed: " << e.what() << std::endl;
        peripheral.disconnect();
        return EXIT_FAILURE;
    }

    if (!subscribed) {
        peripheral.disconnect();
        return EXIT_FAILURE;
    }

    std::cout << (used_indicate ? "Indication" : "Notification")
              << " active on characteristic " << characteristic_uuid
              << ". Press Enter to stop..." << std::endl;

    if (std::cin.peek() == '\n') std::cin.get();
    std::string line;
    std::getline(std::cin, line);

    try {
        peripheral.unsubscribe(service_uuid, characteristic_uuid);
    } catch (const std::exception& e) {
        std::cerr << "Unsubscribe failed (continuing): " << e.what() << std::endl;
    }

    peripheral.disconnect();
    std::cout << "Disconnected. Exiting." << std::endl;
    return EXIT_SUCCESS;
}
