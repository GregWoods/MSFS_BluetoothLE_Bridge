#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_set>

#include "simpleble/SimpleBLE.h"

// Shared helpers
std::optional<SimpleBLE::Adapter> get_first_adapter();
std::string to_lower(std::string v);
bool find_characteristic(SimpleBLE::Peripheral& p,
                         const std::string& desired_lower,
                         SimpleBLE::BluetoothUUID& out_service,
                         SimpleBLE::BluetoothUUID& out_char);

// Session that scans until all provided MAC addresses are found, then connects/subscribes all.
int ble_run_session_scan_until_all_addresses(
    const std::unordered_set<std::string>& addresses_lower,
    const std::string& characteristic_uuid,
    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet);