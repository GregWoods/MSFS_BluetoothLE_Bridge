#include <array>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <fstream>
#include <cctype>
#include <filesystem>

#include "simpleble/SimpleBLE.h"
#include "WASMIF.h"

// Command-line selectable config; defaults to "default.config" if not provided via --config
constexpr const char* DEFAULT_CONFIG_FILENAME = "default.config";
constexpr const char* BLE_STRING_IDENTIFIER = "SHB1000";
constexpr const char* BLE_CHARACTERISTIC_UUID = "f62a9f56-f29e-48a8-a317-47ee37a58999";
constexpr int BLUETOOTH_SCANNING_TIMEOUT_SEC = 22;

static_assert(sizeof(void*) == 8, "Must build 64-bit (x64).");

static WASMIF* wasmPtr = nullptr;

// Staging buffer required by WASMIF (max length enforced by MAX_CALC_CODE_SIZE)
static std::array<char, MAX_CALC_CODE_SIZE> ccode{};

// Simple 256-entry lookup table mapping byte -> calculator code
static std::array<std::string, 256> g_codeMap{};

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

    size_t countAssigned = 0, maxLen = 0;
    std::string line;
    size_t lineNum = 0;

    while (std::getline(in, line)) {
        ++lineNum;
        std::string raw = trim(line);
        if (raw.empty()) continue;
        // Skip comments and section headers like [88:...]
        if (raw.rfind("//", 0) == 0 || raw.rfind("#", 0) == 0 || raw.rfind(";", 0) == 0 || raw.front() == '[') continue;

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

        bool wasEmpty = g_codeMap[key].empty();
        g_codeMap[key] = std::move(value); // may be empty for 'unassigned'

        if (!g_codeMap[key].empty()) {
            if (wasEmpty) ++countAssigned;
            maxLen = std::max(maxLen, g_codeMap[key].size());
        }
    }

    std::cout << "Loaded " << countAssigned << " code mappings from " << path
              << " (max length " << maxLen << ").\n";
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

// Shared session runner implemented in src/ble_session.cpp
int ble_run_session(const std::string& device_identifier,
                    const std::string& characteristic_uuid,
                    int scan_timeout_sec,
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
        // Marker appended to console output if code is not sent to the sim
        const std::string sentNote = wasmPtr ? "" : " Not Sent";

        for (uint8_t b : bytes) {
            const std::string& code = g_codeMap[b];
            // Log the received byte and the calculator code being sent (or not)
            std::cout << "Device: " << devTag << " InputByte: 0x"
                        << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                        << static_cast<int>(b)
                        << std::dec << std::nouppercase << std::setfill(' ')
                        << " Output: " << code << sentNote << std::endl;

            send_calc_code_safely(code);
        }
    };

    return ble_run_session(
        BLE_STRING_IDENTIFIER,
        BLE_CHARACTERISTIC_UUID,
        BLUETOOTH_SCANNING_TIMEOUT_SEC,
        on_packet
    );
}