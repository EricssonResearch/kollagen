cmake_minimum_required(VERSION 3.0.2)

project(kollagen)

include (CTest)

set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT MSVC)
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
    set(CMAKE_CXX_FLAGS_Release "-DNDEBUG -O3")
else()
    set(CMAKE_CXX_FLAGS "/utf-8 /D_USE_MATH_DEFINES /Ox")
endif()

option(KOLLAGEN_ENABLE_TESTS "Run the tests" OFF)
option(KOLLAGEN_ENABLE_GTSAM "Enable GTSAM" OFF)
option(KOLLAGEN_BUILD_EXAMPLES "Build examples" ON)

list(APPEND KOLLAGEN_INCLUDE_DIRS "include")

if(KOLLAGEN_ENABLE_GTSAM)
    find_package(GTSAM REQUIRED)
    list(APPEND KOLLAGEN_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR} )
    add_executable(iSAM2example examples/iSAM2example.cpp)
    configure_file(examples/data/iSAM2example.json ${CMAKE_CURRENT_BINARY_DIR}/data/iSAM2example.json COPYONLY)
    target_link_libraries(iSAM2example gtsam)
endif()

include_directories(
    ${KOLLAGEN_INCLUDE_DIRS}
)

if(KOLLAGEN_BUILD_EXAMPLES)
configure_file(examples/data/example.json ${CMAKE_CURRENT_BINARY_DIR}/data/example.json COPYONLY)
add_executable(example examples/example.cpp)
add_executable(timing examples/timing.cpp)
endif()

add_executable(generate src/generate.cpp)

if(NOT MSVC)
    add_custom_target(${PROJECT_NAME}-symlink ALL ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/generate ${CMAKE_SOURCE_DIR}/generate DEPENDS generate)
endif()

if(BUILD_TESTING AND KOLLAGEN_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
