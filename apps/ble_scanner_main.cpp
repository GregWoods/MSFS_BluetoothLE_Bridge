#include <functional>
#include <iomanip>
#include <iostream>
#include <string>

#include "simpleble/SimpleBLE.h"
#include "ble_session.h"



int ble_run_session(const std::string& device_identifier,
    const std::string& characteristic_uuid,
    int scan_timeout_sec,
    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet) {
    const std::string desired_char_lower = to_lower(characteristic_uuid);
    

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
    adapter.set_callback_on_scan_start([]() { std::cout << "Scan started." << std::endl; });
    adapter.set_callback_on_scan_stop([]() { std::cout << "Scan stopped." << std::endl; });

    adapter.scan_for(scan_timeout_sec * 1000);

    if (scanned.empty()) {
        std::cerr << "No connectable peripherals discovered." << std::endl;
        return EXIT_FAILURE;
    }

    // Filter by identifier if provided (non-empty)
    std::vector<SimpleBLE::Peripheral> targets;
    if (!device_identifier.empty()) {
        for (auto& p : scanned) {
            if (p.identifier() == device_identifier) targets.push_back(p);
        }
    }
    else {
        targets = scanned;
    }

    if (targets.empty()) {
        std::cerr << "No matching devices found." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Connecting to " << targets.size() << " device(s)..." << std::endl;

    struct ConnectedDevice {
        SimpleBLE::Peripheral peripheral;
        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID characteristic_uuid;
        bool used_indicate = false;
    };

    std::vector<ConnectedDevice> connected;
    connected.reserve(targets.size());

    for (auto& p : targets) {
        std::cout << "-> Connecting: " << p.identifier()
            << " [" << p.address() << "]" << std::endl;
        try {
            p.connect();
        }
        catch (const std::exception& e) {
            std::cerr << "   Connection failed: " << e.what() << " (skipping)" << std::endl;
            continue;
        }

        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID char_uuid;
        if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
            std::cerr << "   Target characteristic not found on "
                << p.address() << " (skipping)" << std::endl;
            try { p.disconnect(); }
            catch (...) {}
            continue;
        }

        bool subscribed = false;
        bool used_indicate = false;

        try {
            bool can_indicate = false;
            bool can_notify = false;

            // Inspect characteristic properties
            for (auto& service : p.services()) {
                if (service.uuid() != service_uuid) continue;
                for (auto& chr : service.characteristics()) {
                    if (chr.uuid() != char_uuid) continue;
                    can_indicate = chr.can_indicate();
                    can_notify = chr.can_notify();
                    break;
                }
            }

            auto devTag = p.address();
            if (can_indicate) {
                p.indicate(service_uuid, char_uuid,
                    [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                        on_packet(bytes, devTag);
                    });
                subscribed = true;
                used_indicate = true;
            }
            else if (can_notify) {
                p.notify(service_uuid, char_uuid,
                    [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                        on_packet(bytes, devTag);
                    });
                subscribed = true;
            }
            else {
                std::cerr << "   Char supports neither indicate nor notify on "
                    << devTag << " (skipping)" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "   Subscription failed: " << e.what() << " (disconnecting)" << std::endl;
            try { p.disconnect(); }
            catch (...) {}
            subscribed = false;
        }

        if (subscribed) {
            std::cout << "   " << (used_indicate ? "Indication" : "Notification")
                << " active on " << p.address() << std::endl;
            connected.push_back({ p, service_uuid, char_uuid, used_indicate });
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
        }
        catch (...) {}
        try {
            cd.peripheral.disconnect();
        }
        catch (...) {}
    }

    std::cout << "Disconnected all. Exiting." << std::endl;
    return EXIT_SUCCESS;
}



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