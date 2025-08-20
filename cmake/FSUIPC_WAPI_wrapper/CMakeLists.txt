cmake_minimum_required(VERSION 3.15)

# Wrapper to build FSUIPC_WAPI sources (Visual Studio .vcxproj replaced by CMake).

# FSUIPC_WAPI CMake wrapper (keeps submodule pristine)

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "FSUIPC_WAPI wrapper: x86 build unsupported.")
endif()

# -------- SimConnect SDK discovery --------
# Expect SIMCONNECT_SDK_ROOT to contain:
#   include/SimConnect.h
#   SimConnect SDK/lib/static/SimConnect.lib
#   SimConnect SDK/lib/static/SimConnect_debug.lib
#
# You can pass -DSIMCONNECT_SDK_ROOT="C:/MSFS SDK" or set it in CMakePresets.json.
# Adjust fallback if you want an auto default.

if (NOT SIMCONNECT_SDK_ROOT)
    # (Optional) try a conventional default; comment out if unwanted.
    set(_sim_default "C:/MSFS 2024 SDK")
    if (EXISTS "${_sim_default}/SimConnect SDK/include/SimConnect.h")
        set(SIMCONNECT_SDK_ROOT "${_sim_default}" CACHE PATH "Root of MSFS SDK (parent of 'SimConnect SDK')" FORCE)
    endif()
endif()

if (NOT SIMCONNECT_SDK_ROOT)
    message(FATAL_ERROR "SIMCONNECT_SDK_ROOT not set. Provide -DSIMCONNECT_SDK_ROOT=\"C:/Path/To/MSFS SDK\"")
endif()

set(_SIM_INC_DIR "${SIMCONNECT_SDK_ROOT}/SimConnect SDK/include")
set(_SIM_LIB_DIR "${SIMCONNECT_SDK_ROOT}/SimConnect SDK/lib/static")

if (NOT EXISTS "${_SIM_INC_DIR}/SimConnect.h")
    message(FATAL_ERROR "SimConnect.h not found in ${_SIM_INC_DIR}")
endif()
if (NOT EXISTS "${_SIM_LIB_DIR}")
    message(FATAL_ERROR "SimConnect lib directory not found: ${_SIM_LIB_DIR}")
endif()

# Find libraries (Debug vs Release)
find_library(SIMCONNECT_LIB_DEBUG
    NAMES SimConnect_debug
    PATHS "${_SIM_LIB_DIR}"
    NO_DEFAULT_PATH
)
find_library(SIMCONNECT_LIB_RELEASE
    NAMES SimConnect
    PATHS "${_SIM_LIB_DIR}"
    NO_DEFAULT_PATH
)

if (NOT SIMCONNECT_LIB_DEBUG)
    message(FATAL_ERROR "SimConnect_debug.lib not found in ${_SIM_LIB_DIR}")
endif()
if (NOT SIMCONNECT_LIB_RELEASE)
    message(FATAL_ERROR "SimConnect.lib not found in ${_SIM_LIB_DIR}")
endif()

message(STATUS "SimConnect include: ${_SIM_INC_DIR}")
message(STATUS "SimConnect (Debug) : ${SIMCONNECT_LIB_DEBUG}")
message(STATUS "SimConnect (Rel)   : ${SIMCONNECT_LIB_RELEASE}")

# Create imported target with per-config locations (works for single or multi-config generators)
add_library(SimConnect::SimConnect UNKNOWN IMPORTED)
set_target_properties(SimConnect::SimConnect PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_SIM_INC_DIR}"
    IMPORTED_LOCATION_DEBUG "${SIMCONNECT_LIB_DEBUG}"
    IMPORTED_LOCATION_RELWITHDEBINFO "${SIMCONNECT_LIB_RELEASE}"
    IMPORTED_LOCATION_RELEASE "${SIMCONNECT_LIB_RELEASE}"
    IMPORTED_LOCATION_MINSIZEREL "${SIMCONNECT_LIB_RELEASE}"
)

# For single-config generators (Ninja) CMAKE_BUILD_TYPE selects the correct location automatically.

# -------- FSUIPC_WAPI sources --------
set(FSUIPC_WAPI_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/../../libs/FSUIPC_WAPI/FSUIPC_WAPI")

file(GLOB FSUIPC_WAPI_SOURCES CONFIGURE_DEPENDS
    "${FSUIPC_WAPI_SRC_DIR}/*.cpp"
)

if (FSUIPC_WAPI_SOURCES STREQUAL "")
    message(FATAL_ERROR "No FSUIPC_WAPI sources found in ${FSUIPC_WAPI_SRC_DIR}")
endif()

add_library(FSUIPC_WAPI STATIC ${FSUIPC_WAPI_SOURCES})

target_include_directories(FSUIPC_WAPI PUBLIC
    "${FSUIPC_WAPI_SRC_DIR}"
)

# Map legacy 'boolean' token to standard bool without touching submodule sources.
target_compile_definitions(FSUIPC_WAPI PRIVATE boolean=bool)

target_compile_definitions(FSUIPC_WAPI
    PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE
    PRIVATE _CRT_SECURE_NO_WARNINGS
)

# Link against imported SimConnect
target_link_libraries(FSUIPC_WAPI PUBLIC SimConnect::SimConnect Shlwapi)

add_library(FSUIPC_WAPI::FSUIPC_WAPI ALIAS FSUIPC_WAPI)
