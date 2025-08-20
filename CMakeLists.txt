cmake_minimum_required(VERSION 3.21)

project(msfs_ble_g1000 LANGUAGES CXX)

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message(FATAL_ERROR "x86 build not supported. Reconfigure with x64 toolchain.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT ipo_msg)
    if (ipo_supported)
        message(STATUS "IPO/LTO enabled")
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    endif()
endif()

add_subdirectory(libs/simpleble/simpleble)
add_subdirectory(cmake/FSUIPC_WAPI_wrapper)

add_executable(msfs_ble_g1000 src/simionic.cpp)

target_link_libraries(msfs_ble_g1000 PRIVATE
    simpleble::simpleble
    FSUIPC_WAPI::FSUIPC_WAPI
)

if (MSVC AND CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(msfs_ble_g1000 PRIVATE /O2 /GL)
    target_link_options(msfs_ble_g1000 PRIVATE /LTCG)
endif()

if (MSVC)
    target_compile_definitions(msfs_ble_g1000 PRIVATE UNICODE _UNICODE)
endif()

install(TARGETS msfs_ble_g1000 RUNTIME DESTINATION .)