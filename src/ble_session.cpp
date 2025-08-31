#include <algorithm>
#include <cctype>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <mutex>

#include "simpleble/SimpleBLE.h"

namespace {

std::optional<SimpleBLE::Adapter> get_first_adapter() {
    try {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) return std::nullopt;
        return adapters.front();
    } catch (const std::exception& e) {
        std::cerr << "Adapter enumeration failed: " << e.what() << std::endl;
        return std::nullopt;
    }
}

std::string to_lower(std::string v) {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}

bool find_characteristic(SimpleBLE::Peripheral& p,
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

} // namespace

// Shared session runner. Declaration is forward-declared in each main.
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
    } else {
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

            // Inspect characteristic properties
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
                           [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                               on_packet(bytes, devTag);
                           });
                subscribed = true;
                used_indicate = true;
            } else if (can_notify) {
                p.notify(service_uuid, char_uuid,
                         [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                             on_packet(bytes, devTag);
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


// New: continuously scan until all target MACs are found and subscribed.
int ble_run_session_scan_until_all_addresses(
    const std::unordered_set<std::string>& addresses_lower,
    const std::string& characteristic_uuid,
    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet) {

    if (addresses_lower.empty()) {
        std::cerr << "No target MAC addresses provided." << std::endl;
        return EXIT_FAILURE;
    }

    const std::string desired_char_lower = to_lower(characteristic_uuid);

    auto adapter_opt = get_first_adapter();
    if (!adapter_opt) {
        std::cerr << "No Bluetooth adapter found." << std::endl;
        return EXIT_FAILURE;
    }
    auto adapter = *adapter_opt;

    // Queue for discovered target peripherals; protect with a mutex (scanning callback thread)
    std::mutex qmtx;
    std::vector<SimpleBLE::Peripheral> discovered_queue;
    std::unordered_set<std::string> pending; // macs currently queued for processing

    struct ConnectedDevice {
        SimpleBLE::Peripheral peripheral;
        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID characteristic_uuid;
        bool used_indicate = false;
    };
    std::unordered_map<std::string, ConnectedDevice> connected; // key: mac lower
    connected.reserve(addresses_lower.size());

    adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral p) {
        if (!p.is_connectable()) return;
        const std::string addr_lower = to_lower(p.address());
        // Only consider configured targets not yet connected
        if (addresses_lower.count(addr_lower) == 0) return;
        if (connected.find(addr_lower) != connected.end()) return;

        {
            std::lock_guard<std::mutex> lock(qmtx);
            // Avoid flooding the queue with duplicates for the same MAC
            if (pending.insert(addr_lower).second) {
                discovered_queue.push_back(p);
            }
        }
        std::cout << "Found target: " << p.identifier() << " [" << p.address() << "]" << std::endl;
    });

    adapter.set_callback_on_scan_start([]() { std::cout << "Scan started (continuous)..." << std::endl; });
    adapter.set_callback_on_scan_stop([]() { std::cout << "Scan stopped." << std::endl; });

    // Start continuous scan (no timeout)
    try {
        adapter.scan_start();
    } catch (const std::exception& e) {
        std::cerr << "Scan start failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Process discoveries and connect until all are subscribed
    while (connected.size() < addresses_lower.size()) {
        std::vector<SimpleBLE::Peripheral> to_process;
        {
            std::lock_guard<std::mutex> lock(qmtx);
            to_process.swap(discovered_queue);
        }

        for (auto& p : to_process) {
            const std::string mac = to_lower(p.address());
            {
                std::lock_guard<std::mutex> lock(qmtx);
                pending.erase(mac); // allow re-queue on future rediscovery if needed
            }
            if (connected.find(mac) != connected.end()) continue;

            std::cout << "-> Connecting: " << p.identifier()
                      << " [" << p.address() << "]" << std::endl;

            try {
                p.connect();
            } catch (const std::exception& e) {
                std::cerr << "   Connection failed: " << e.what() << " (will retry when rediscovered)" << std::endl;
                continue;
            }

            SimpleBLE::BluetoothUUID service_uuid;
            SimpleBLE::BluetoothUUID char_uuid;
            if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
                std::cerr << "   Target characteristic not found on "
                          << p.address() << " (disconnecting; will retry when rediscovered)" << std::endl;
                try { p.disconnect(); } catch (...) {}
                continue;
            }

            bool subscribed = false;
            bool used_indicate = false;

            try {
                bool can_indicate = false;
                bool can_notify   = false;

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
                               [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                                   on_packet(bytes, devTag);
                               });
                    subscribed = true;
                    used_indicate = true;
                } else if (can_notify) {
                    p.notify(service_uuid, char_uuid,
                             [on_packet, devTag](SimpleBLE::ByteArray bytes) {
                                 on_packet(bytes, devTag);
                             });
                    subscribed = true;
                } else {
                    std::cerr << "   Char supports neither indicate nor notify on "
                              << devTag << " (disconnecting; will retry)" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "   Subscription failed: " << e.what() << " (disconnecting; will retry)" << std::endl;
            }

            if (subscribed) {
                std::cout << "   " << (used_indicate ? "Indication" : "Notification")
                          << " active on " << p.address() << std::endl;
                connected.emplace(mac, ConnectedDevice{p, service_uuid, char_uuid, used_indicate});
            } else {
                try { p.disconnect(); } catch (...) {}
            }
        }

        // Small idle delay; scanning continues in background
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // All connected; stop scanning now
    try { adapter.scan_stop(); } catch (...) {}

    std::cout << "Listening on " << connected.size()
              << " device(s). Press Enter to stop..." << std::endl;
    if (std::cin.peek() == '\n') std::cin.get();
    std::string line;
    std::getline(std::cin, line);

    // Cleanup
    for (auto& kv : connected) {
        auto& cd = kv.second;
        try { cd.peripheral.unsubscribe(cd.service_uuid, cd.characteristic_uuid); } catch (...) {}
        try { cd.peripheral.disconnect(); } catch (...) {}
    }

    std::cout << "Disconnected all. Exiting." << std::endl;
    return EXIT_SUCCESS;
}