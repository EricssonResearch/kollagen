cmake_minimum_required (VERSION 3.0.2)

project(data_generator)

include (CTest)

set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_Release "-DNDEBUG -O3")

option(DATAGEN_ENABLE_TESTS "Run the tests" OFF)
option(DATAGEN_ENABLE_GTSAM "Enable GTSAM" OFF)

list(APPEND DATAGEN_INCLUDE_DIRS "include")

if(DATAGEN_ENABLE_GTSAM)
    find_package(GTSAM REQUIRED)
    list(APPEND DATAGEN_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR} )
    add_executable(iSAM2example src/iSAM2example.cpp)
    target_link_libraries(iSAM2example gtsam)
endif()

include_directories(
    ${DATAGEN_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
add_executable(timing src/timing.cpp)
add_executable(generate src/generate.cpp)

add_custom_target(${PROJECT_NAME}-symlink ALL ln --force -s ${CMAKE_CURRENT_BINARY_DIR}/generate ${CMAKE_SOURCE_DIR}/generate DEPENDS generate)

if(BUILD_TESTING AND DATAGEN_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
