cmake_minimum_required(VERSION 3.21)

project(msfs_ble_g1000 LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# (Optional) Force static MSVC runtime so no redist is needed.
# Works with CMake 3.15+
if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# (Optional) Enable LTO for smaller/faster Release builds (needs proper toolchain support)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT ipo_msg)
    if (ipo_supported)
        message(STATUS "IPO/LTO enabled")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    else()
        message(STATUS "IPO/LTO not supported: ${ipo_msg}")
    endif()
endif()

add_subdirectory(libs/simpleble/simpleble) 

add_executable(msfs_ble_g1000
    src/simionic.cpp
)

# Link against the SimpleBLE target
target_link_libraries(msfs_ble_g1000 PRIVATE simpleble::simpleble)

# (Optional) Strip symbols on Release (MSVC: /DEBUG not generated anyway unless /Zi)
if (MSVC AND CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(msfs_ble_g1000 PRIVATE /O2 /GL)
    target_link_options(msfs_ble_g1000 PRIVATE /LTCG)
endif()

# On Windows, ensure Unicode (optional)
if (MSVC)
    target_compile_definitions(msfs_ble_g1000 PRIVATE UNICODE _UNICODE)
endif()

# Installation (optional if you want "cmake --install")
install(TARGETS msfs_ble_g1000 RUNTIME DESTINATION .)

#if (CMAKE_SIZEOF_VOID_P EQUAL 4)
#    message(FATAL_ERROR "x86 build not supported. Use 64-bit toolchain/preset.")
#endif()