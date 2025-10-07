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
#include "ble_session.h"

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

// New: continuously scan until all target MACs are found, then connect/subscribe all.
// No data is received (no subscription) until discovery is complete.
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

    // Phase 1: Discovery only (no connections/subscriptions yet)
    std::mutex mtx;
    std::unordered_map<std::string, SimpleBLE::Peripheral> discovered; // mac -> peripheral
    discovered.reserve(addresses_lower.size());

    adapter.set_callback_on_scan_found([&](SimpleBLE::Peripheral p) {
        if (!p.is_connectable()) return;
        const std::string mac = to_lower(p.address());
        if (addresses_lower.count(mac) == 0) return; // not a target
        {
            std::lock_guard<std::mutex> lk(mtx);
            if (discovered.find(mac) == discovered.end()) {
                std::cout << "Found target: " << p.identifier()
                          << " [" << p.address() << "] ("
                          << (discovered.size() + 1) << "/" << addresses_lower.size()
                          << ")" << std::endl;
            }
            discovered[mac] = p;
        }
    });

    adapter.set_callback_on_scan_start([]() { std::cout << "Scan started (discovery phase)..." << std::endl; });
    adapter.set_callback_on_scan_stop([]() { std::cout << "Scan stopped (discovery complete)." << std::endl; });

    try {
        adapter.scan_start();
    } catch (const std::exception& e) {
        std::cerr << "Scan start failed: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    while (true) {
        {
            std::lock_guard<std::mutex> lk(mtx);
            if (discovered.size() >= addresses_lower.size()) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    try { adapter.scan_stop(); } catch (...) {}

    struct ConnectedDevice {
        SimpleBLE::Peripheral peripheral;
        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID characteristic_uuid;
        bool used_indicate = false;
    };
    std::unordered_map<std::string, ConnectedDevice> connected;
    connected.reserve(addresses_lower.size());

    for (const auto& mac : addresses_lower) {
        auto it = discovered.find(mac);
        if (it == discovered.end()) {
            std::cerr << "Internal error: target " << mac << " missing from discovery set." << std::endl;
            for (auto& kv : connected) {
                try { kv.second.peripheral.unsubscribe(kv.second.service_uuid, kv.second.characteristic_uuid); } catch (...) {}
                try { kv.second.peripheral.disconnect(); } catch (...) {}
            }
            return EXIT_FAILURE;
        }

        auto p = it->second;
        std::cout << "-> Connecting: " << p.identifier()
                  << " [" << p.address() << "]" << std::endl;

        try {
            p.connect();
        } catch (const std::exception& e) {
            std::cerr << "   Connection failed: " << e.what() << std::endl;
            for (auto& [mac2, cd] : connected) {
                try { cd.peripheral.unsubscribe(cd.service_uuid, cd.characteristic_uuid); } catch (...) {}
                try { cd.peripheral.disconnect(); } catch (...) {}
            }
            return EXIT_FAILURE;
        }

        SimpleBLE::BluetoothUUID service_uuid;
        SimpleBLE::BluetoothUUID char_uuid;
        if (!find_characteristic(p, desired_char_lower, service_uuid, char_uuid)) {
            std::cerr << "   Target characteristic not found on " << p.address() << std::endl;
            try { p.disconnect(); } catch (...) {}
            for (auto& [mac2, cd] : connected) {
                try { cd.peripheral.unsubscribe(cd.service_uuid, cd.characteristic_uuid); } catch (...) {}
                try { cd.peripheral.disconnect(); } catch (...) {}
            }
            return EXIT_FAILURE;
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
                std::cerr << "   Char supports neither indicate nor notify on " << devTag << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "   Subscription failed: " << e.what() << std::endl;
        }

        if (!subscribed) {
            try { p.disconnect(); } catch (...) {}
            for (auto& kv : connected) {
                try { kv.second.peripheral.unsubscribe(kv.second.service_uuid, kv.second.characteristic_uuid); } catch (...) {}
                try { kv.second.peripheral.disconnect(); } catch (...) {}
            }
            return EXIT_FAILURE;
        }

        std::cout << "   " << (used_indicate ? "Indication" : "Notification")
                  << " active on " << p.address()
                  << " (" << (connected.size() + 1) << "/" << addresses_lower.size() << ")"
                  << std::endl;

        connected.emplace(mac, ConnectedDevice{p, service_uuid, char_uuid, used_indicate});
    }

    std::cout << "All targets connected. Listening on " << connected.size()
              << " device(s). Press Enter to stop..." << std::endl;
    if (std::cin.peek() == '\n') std::cin.get();
    std::string line;
    std::getline(std::cin, line);

    for (auto& kv : connected) {
        auto& cd = kv.second;
        try { cd.peripheral.unsubscribe(cd.service_uuid, cd.characteristic_uuid); } catch (...) {}
        try { cd.peripheral.disconnect(); } catch (...) {}
    }

    std::cout << "Disconnected all. Exiting." << std::endl;
    return EXIT_SUCCESS;
}