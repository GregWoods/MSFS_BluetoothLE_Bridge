#include <array>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <cctype>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>

#include "simpleble/SimpleBLE.h"
#include "WASMIF.h"

// Command-line selectable settings file; defaults to "default.settings" if not provided via --config
constexpr const char* DEFAULT_CONFIG_FILENAME = "default.settings";
constexpr const char* BLE_STRING_IDENTIFIER = "SHB1000";
constexpr const char* BLE_CHARACTERISTIC_UUID = "f62a9f56-f29e-48a8-a317-47ee37a58999";
constexpr int BLUETOOTH_SCANNING_TIMEOUT_SEC = 27;

static_assert(sizeof(void*) == 8, "Must build 64-bit (x64).");

static WASMIF* wasmPtr = nullptr;

// Staging buffer required by WASMIF (max length enforced by MAX_CALC_CODE_SIZE)
static std::array<char, MAX_CALC_CODE_SIZE> ccode{};

// Per-device 256-entry lookup table mapping byte -> calculator code
struct DeviceConfig {
    std::array<std::string, 256> codes{};
    size_t assignedCount = 0;
    size_t maxLen = 0;
};

// Registry of device configs keyed by canonical MAC (lowercase, colon-separated)
static std::unordered_map<std::string, DeviceConfig> g_deviceMaps;

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

static std::string trim(std::string s) {
    auto isspace2 = [](unsigned char ch){ return std::isspace(ch) != 0; };
    size_t start = 0;
    while (start < s.size() && isspace2(static_cast<unsigned char>(s[start]))) ++start;
    size_t end = s.size();
    while (end > start && isspace2(static_cast<unsigned char>(s[end - 1]))) --end;
    return s.substr(start, end - start);
}

static bool iequals_ascii(const std::string& a, const char* b) {
    size_t n = a.size();
    size_t m = std::strlen(b);
    if (n != m) return false;
    for (size_t i = 0; i < n; ++i) {
        unsigned char ca = static_cast<unsigned char>(a[i]);
        unsigned char cb = static_cast<unsigned char>(b[i]);
        if (std::tolower(ca) != std::tolower(cb)) return false;
    }
    return true;
}

static bool is_hex_char(unsigned char c) {
    return std::isxdigit(c) != 0;
}

// Convert input to canonical MAC "aa:bb:cc:dd:ee:ff" (lowercase).
// Accepts formats like "AA:BB:CC:DD:EE:FF", "aa-bb-cc-dd-ee-ff", or "AABBCCDDEEFF".
static bool canonicalize_mac(const std::string& in, std::string& out) {
    // Strip non-hex chars
    std::string hex;
    hex.reserve(12);
    for (unsigned char c : in) {
        if (is_hex_char(c)) hex.push_back(static_cast<char>(std::tolower(c)));
    }
    if (hex.size() != 12) return false;
    // Insert colons
    out.clear();
    out.reserve(17);
    for (size_t i = 0; i < 12; ++i) {
        out.push_back(hex[i]);
        if (i % 2 == 1 && i != 11) out.push_back(':');
    }
    return true;
}

static bool parse_config_line(const std::string& line, uint8_t& outKey, std::string& outValue) {
    // Expected: <HEX> = "<calculator code>" OR <HEX> = unassigned
    auto posEq = line.find('=');
    if (posEq == std::string::npos) return false;

    std::string left = trim(line.substr(0, posEq));
    std::string right = trim(line.substr(posEq + 1));
    if (left.empty() || right.empty()) return false;

    // Parse left as hex byte (allow "A8" or "0xA8")
    try {
        unsigned long v = std::stoul(left, nullptr, 16);
        if (v > 0xFFul) return false;
        outKey = static_cast<uint8_t>(v & 0xFFu);
    } catch (...) {
        return false;
    }

    // Handle unassigned (case-insensitive, unquoted)
    if (iequals_ascii(right, "unassigned")) {
        outValue.clear(); // leave empty mapping
        return true;
    }

    // Otherwise expect a quoted value
    size_t firstQuote = right.find('"');
    if (firstQuote == std::string::npos) return false;
    size_t lastQuote = right.rfind('"');
    if (lastQuote == std::string::npos || lastQuote <= firstQuote) return false;

    outValue = right.substr(firstQuote + 1, lastQuote - firstQuote - 1);
    return true;
}

static void load_code_map_from_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Config not found: " << path << ". No code mappings loaded.\n";
        return;
    }

    std::string currentSection; // canonical MAC section
    size_t totalSections = 0;
    std::string line;
    size_t lineNum = 0;

    while (std::getline(in, line)) {
        ++lineNum;
        std::string raw = trim(line);
        if (raw.empty()) continue;

        // Skip comments
        if (raw.rfind("//", 0) == 0 || raw.rfind("#", 0) == 0 || raw.rfind(";", 0) == 0) continue;

        // Section header [ ... ]
        if (raw.front() == '[' && raw.back() == ']') {
            std::string sectInner = trim(raw.substr(1, raw.size() - 2));
            std::string canon;
            if (canonicalize_mac(sectInner, canon)) {
                currentSection = canon;
                // Ensure device exists
                if (g_deviceMaps.find(currentSection) == g_deviceMaps.end()) {
                    g_deviceMaps.emplace(currentSection, DeviceConfig{});
                }
                ++totalSections;
            } else {
                std::cerr << "Warning: invalid section header on line " << lineNum << ": " << line << "\n";
                currentSection.clear();
            }
            continue;
        }

        // Ignore mapping lines outside any device section
        if (currentSection.empty()) continue;

        uint8_t key{};
        std::string value;
        if (!parse_config_line(raw, key, value)) {
            std::cerr << "Warning: invalid config line " << lineNum << ": " << line << "\n";
            continue;
        }

        // Only enforce size for assigned (non-empty) mappings
        if (!value.empty() && value.size() >= MAX_CALC_CODE_SIZE) {
            std::cerr << "Warning: mapping for 0x" << std::hex << std::uppercase
                      << static_cast<int>(key) << std::dec << std::nouppercase
                      << " exceeds MAX_CALC_CODE_SIZE (" << MAX_CALC_CODE_SIZE
                      << "). It will be truncated.\n";
            value.resize(MAX_CALC_CODE_SIZE - 1);
        }

        // Store into the current device's table
        DeviceConfig& target = g_deviceMaps[currentSection];
        const bool wasEmpty = target.codes[key].empty();
        target.codes[key] = std::move(value);
        if (!target.codes[key].empty()) {
            if (wasEmpty) ++target.assignedCount;
            target.maxLen = std::max(target.maxLen, target.codes[key].size());
        }
    }

    std::cout << "Config load summary: " << totalSections << " device section(s).\n";
    for (const auto& kv : g_deviceMaps) {
        std::cout << " - [" << kv.first << "]: " << kv.second.assignedCount
                  << " mapping(s), max length " << kv.second.maxLen << "\n";
    }
}

// Resolve config path for the given filename:
// - In Release: use current working directory only.
// - In Debug: also check exactly two directories up from CWD.
static std::string resolve_config_path(const std::string& filename) {
    namespace fs = std::filesystem;
    std::error_code ec;

    const fs::path cwd = fs::current_path(ec);
    if (ec) return {};

    fs::path candidate = cwd / filename;
    if (fs::exists(candidate, ec) && !ec) return candidate.string();

#if defined(_DEBUG)
    fs::path up2 = cwd.parent_path().parent_path();
    candidate = up2 / filename;
    if (fs::exists(candidate, ec) && !ec) return candidate.string();
#endif

    return {};
}

static void send_calc_code_safely(const std::string& code) {
    if (!wasmPtr || code.empty()) return;
    // Copy into fixed buffer to guarantee null-termination and size limit
    strncpy_s(ccode.data(), ccode.size(), code.c_str(), _TRUNCATE);
    wasmPtr->executeCalclatorCode(ccode.data());
}

// Helper: log "Device: [tag] Input: XX Output: <text><suffix>" with proper hex formatting
static void log_device_input_output(const std::string& devTag, uint8_t byte, const std::string& output, const std::string& suffix) {
    std::cout << "Device: [" << devTag << "] Input: "
              << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << static_cast<int>(byte)
              << std::dec << std::nouppercase << std::setfill(' ')
              << " Output: " << output << " " << suffix << " " << std::endl;
}

// Continuous scan until all target MACs are connected (implemented in src/ble_session.cpp)
int ble_run_session_scan_until_all_addresses(
    const std::unordered_set<std::string>& addresses_lower,
    const std::string& characteristic_uuid,
    const std::function<void(const SimpleBLE::ByteArray&, const std::string&)>& on_packet);

// Parse --config=<file> or --config <file>; return chosen filename (defaults to DEFAULT_CONFIG_FILENAME)
static std::string parse_config_arg(int argc, char* argv[]) {
    std::string cfg = DEFAULT_CONFIG_FILENAME;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 < argc) {
                cfg = argv[++i];
            } else {
                std::cerr << "Warning: --config provided without filename. Using default '" << cfg << "'.\n";
            }
        } else if (arg.rfind("--config=", 0) == 0) {
            cfg = arg.substr(std::string("--config=").size());
        }
    }
    return cfg;
}

// Attempt to canonicalize a device tag (devTag) into a MAC key
static std::string devtag_to_mac_key(const std::string& devTag) {
    std::string mac;
    if (canonicalize_mac(devTag, mac)) return mac;
    // If devTag is not directly a MAC, leave empty; caller can decide fallback behavior.
    return {};
}

int main(int argc, char* argv[]) {
    // SimConnect / WASM init
    wasmPtr = WASMIF::GetInstance();
    WASMIFGuard guard(wasmPtr);
    wasmPtr->setSimConfigConnection(SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
    wasmPtr->start();

    // Choose config filename from command line or default
    const std::string cfgName = parse_config_arg(argc, argv);

    // Resolve and load config
    const std::string cfgPath = resolve_config_path(cfgName);
    if (cfgPath.empty()) {
        std::cerr << "Config '" << cfgName << "' not found. CWD="
                  << std::filesystem::current_path().string()
#if defined(_DEBUG)
                  << " (also checked two levels up)"
#endif
                  << "\n";
    } else {
        std::cout << "Using config: " << cfgPath << std::endl;
        load_code_map_from_file(cfgPath);
    }

    auto on_packet = [](const SimpleBLE::ByteArray& bytes, const std::string& devTag) {
        // Resolve device key
        const std::string macKey = devtag_to_mac_key(devTag);

        // Select per-device config
        const DeviceConfig* cfg = nullptr;
        if (!macKey.empty()) {
            auto it = g_deviceMaps.find(macKey);
            if (it != g_deviceMaps.end()) cfg = &it->second;
        }

        if (!cfg) {
            // No mapping for this device; still log input
            for (uint8_t b : bytes) {
                log_device_input_output(devTag, b, "<no mapping>", "");
            }
            return;
        }

        for (uint8_t b : bytes) {
            const std::string& code = cfg->codes[b];
            if (!code.empty()) {
                log_device_input_output(devTag, b, code, wasmPtr ? "" : "No connection");
                send_calc_code_safely(code);
            } else {
                std::cout << "[" << devTag << "] BYTE: 0x"
                          << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                          << static_cast<int>(b)
                          << std::dec << std::nouppercase << std::setfill(' ')
                          << " CALC: <unassigned>" << std::endl;
            }
        }
    };

    // Build the target MAC set (keys are already canonicalized lower-case in g_deviceMaps)
    std::unordered_set<std::string> target_macs;
    target_macs.reserve(g_deviceMaps.size());
    for (const auto& kv : g_deviceMaps) target_macs.insert(kv.first);

    // Keep scanning until all configured devices are found and connected
    return ble_run_session_scan_until_all_addresses(
        target_macs,
        BLE_CHARACTERISTIC_UUID,
        on_packet
    );
}