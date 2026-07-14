cmake_minimum_required(VERSION 3.15)
project(livox_fast_lio_windows VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── vcpkg toolchain ────────────────────────────────────────────────
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# ─── Build type ─────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ─── Compiler flags ─────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/openmp /EHsc /W3)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -DWIN32_LEAN_AND_MEAN)
else()
    add_compile_options(-O3 -Wall)
    find_package(OpenMP QUIET)
    if(OpenMP_CXX_FOUND)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    endif()
endif()

# ─── Dependencies ───────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(yaml-cpp REQUIRED)

set(FOXGLOVE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/foxglove-sdk/v0.25.2/foxglove")
find_package(foxglove-sdk CONFIG REQUIRED
    PATHS "${FOXGLOVE_SDK_DIR}"
    NO_DEFAULT_PATH)

foxglove_sdk_add_cpp_library(foxglove_cpp
    TYPE SHARED
    REMOTE_ACCESS OFF
)

if(WIN32)
    target_link_libraries(foxglove_cpp PRIVATE ntdll)
endif()

# bzip2 (for reading bz2-compressed bag chunks)
find_package(BZip2 QUIET)
if(NOT BZIP2_FOUND)
    find_path(BZIP2_INCLUDE_DIR NAMES bzlib.h)
    find_library(BZIP2_LIBRARY NAMES bz2)
    if(BZIP2_INCLUDE_DIR AND BZIP2_LIBRARY)
        set(BZIP2_FOUND TRUE)
    endif()
endif()

# zlib (for CRC32, optional)
find_package(ZLIB QUIET)

# lz4 (for reading lz4-compressed bag chunks, optional)
find_path(LZ4_INCLUDE_DIR NAMES lz4.h
    HINTS ${CMAKE_PREFIX_PATH}/include
          ${CMAKE_TOOLCHAIN_FILE}/../installed/x64-windows/include
          ${CMAKE_CURRENT_SOURCE_DIR}/build/vcpkg_installed/x64-windows/include)
find_library(LZ4_LIBRARY NAMES lz4)

# ─── Livox SDK ─────────────────────────────────────────────────────
set(LIVOX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Livox-SDK")
if(EXISTS "${LIVOX_SDK_DIR}/sdk_core/include/livox_sdk.h")
    set(LIVOX_SDK_INCLUDE_DIRS "${LIVOX_SDK_DIR}/sdk_core/include")
    find_library(LIVOX_SDK_LIB
        NAMES livox_sdk_static
        PATHS "${LIVOX_SDK_DIR}/sdk_core/lib/x64"
              "${LIVOX_SDK_DIR}/build/sdk_core/Release"
              "${LIVOX_SDK_DIR}/build/sdk_core/Debug"
        NO_DEFAULT_PATH)
    if(LIVOX_SDK_LIB)
        set(LIVOX_SDK_LIBS ${LIVOX_SDK_LIB})
        message(STATUS "Livox SDK found: ${LIVOX_SDK_LIB}")
    else()
        message(STATUS "Livox SDK headers found; building sdk_core from source.")
        set(_FAST_LIO_PROJECT_NAME "${PROJECT_NAME}")
        set(PROJECT_NAME livox_sdk)
        add_subdirectory("${LIVOX_SDK_DIR}/sdk_core" "${CMAKE_CURRENT_BINARY_DIR}/Livox-SDK/sdk_core" EXCLUDE_FROM_ALL)
        set(PROJECT_NAME "${_FAST_LIO_PROJECT_NAME}")
        set(LIVOX_SDK_LIBS livox_sdk_static)
    endif()
else()
    message(STATUS "Using minimal livox_sdk.h from include/ (SDK not found in third_party/)")
    set(LIVOX_SDK_INCLUDE_DIRS "")
    set(LIVOX_SDK_LIBS "")
endif()

# ─── IKFoM Toolkit (header-only, in third_party/) ──────────────────
# use-ikfom.hpp includes <IKFoM_toolkit/esekfom/esekfom.hpp>
# So the include path must point to third_party/ (parent of IKFoM_toolkit/)
set(IKFOM_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# ─── Include directories ───────────────────────────────────────────
include_directories(
    ${LIVOX_SDK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${IKFOM_INCLUDE}
)

# ─── Root directory definition (runtime path) ──────────────────────
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# ─── Source files ───────────────────────────────────────────────────
set(FAST_LIO_SOURCES
    src/main.cpp
    src/async_map_publisher.cpp
    src/laser_mapping.cpp
    src/preprocess.cpp
    src/livox_adapter.cpp
    src/lvx_reader.cpp
    src/foxglove_publisher.cpp
    src/ros_bag.cpp
    src/bag_reader.cpp
    src/bag_writer.cpp
    src/yaml_config.cpp
    include/ikd-Tree/ikd_Tree.cpp
)

if(LIVOX_SDK_LIBS)
    string(REPLACE "\\" "/" LIVOX_REAL_SDK_HEADER "${LIVOX_SDK_DIR}/sdk_core/include/livox_sdk.h")
else()
    list(APPEND FAST_LIO_SOURCES src/livox_sdk_stub.cpp)
endif()

# ─── Executable ─────────────────────────────────────────────────────
add_executable(livox_fast_lio ${FAST_LIO_SOURCES})

target_link_libraries(livox_fast_lio
    ${PCL_LIBRARIES}
    yaml-cpp::yaml-cpp
    foxglove_cpp
    ${LIVOX_SDK_LIBS}
    ws2_32    # Windows sockets
    bcrypt    # Windows crypto support
)

if(LIVOX_SDK_LIBS)
    target_compile_definitions(livox_fast_lio PRIVATE
        LIVOX_USE_REAL_SDK=1
        LIVOX_REAL_SDK_HEADER="${LIVOX_REAL_SDK_HEADER}")
endif()

# bzip2 for bag decompression
if(BZIP2_FOUND)
    target_include_directories(livox_fast_lio PRIVATE ${BZIP2_INCLUDE_DIR})
    target_link_libraries(livox_fast_lio ${BZIP2_LIBRARY})
    target_compile_definitions(livox_fast_lio PRIVATE HAS_BZIP2=1)
    message(STATUS "BZip2: ${BZIP2_LIBRARY}")
endif()

# zlib (optional, for CRC32)
if(ZLIB_FOUND)
    target_link_libraries(livox_fast_lio ZLIB::ZLIB)
    target_compile_definitions(livox_fast_lio PRIVATE HAS_ZLIB=1)
endif()

# lz4 for bag decompression (optional)
if(LZ4_INCLUDE_DIR AND LZ4_LIBRARY)
    target_include_directories(livox_fast_lio PRIVATE ${LZ4_INCLUDE_DIR})
    target_link_libraries(livox_fast_lio ${LZ4_LIBRARY})
    target_compile_definitions(livox_fast_lio PRIVATE HAS_LZ4=1)
    message(STATUS "LZ4: ${LZ4_LIBRARY}")
endif()

# ─── Install ────────────────────────────────────────────────────────
install(TARGETS livox_fast_lio RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION config)

# ─── Copy config to build directory ─────────────────────────────────
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config/)

enable_testing()

add_executable(livox_fast_lio_tests
    tests/test_livox_fast_lio.cpp
    src/async_map_publisher.cpp
    src/lvx_reader.cpp
    src/yaml_config.cpp
)
target_include_directories(livox_fast_lio_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(livox_fast_lio_tests PRIVATE
    ${PCL_LIBRARIES}
    yaml-cpp::yaml-cpp
    foxglove_cpp
)
add_test(NAME livox_fast_lio_tests COMMAND livox_fast_lio_tests)

message(STATUS "=== livox-fast-lio-windows Build Configuration ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "Livox SDK: ${LIVOX_SDK_INCLUDE_DIRS}")
message(STATUS "Livox SDK libs: ${LIVOX_SDK_LIBS}")
message(STATUS "IKFoM: ${IKFOM_INCLUDE}")
message(STATUS "BZip2: ${BZIP2_FOUND}")
message(STATUS "LZ4 include: ${LZ4_INCLUDE_DIR}")
message(STATUS "LZ4 lib: ${LZ4_LIBRARY}")
