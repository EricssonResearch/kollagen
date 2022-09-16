add_subdirectory(/usr/src/googletest
    ${PROJECT_BINARY_DIR}/gtest
)

set(TEST_BINARY ${PROJECT_NAME}_test)
add_executable(${TEST_BINARY} testWalker.cpp testDataGenerator.cpp testJSONParse.cpp)

target_link_libraries(${TEST_BINARY} gtest gtest_main gmock gmock_main)

set(TEST_DATA_DIR test_data)
configure_file(test_data/walker1.g2o ${TEST_DATA_DIR}/walker1.g2o COPYONLY)
configure_file(test_data/walker2.g2o ${TEST_DATA_DIR}/walker2.g2o COPYONLY)
configure_file(test_data/walker1_3D.g2o ${TEST_DATA_DIR}/walker1_3D.g2o COPYONLY)
configure_file(test_data/walker2_3D.g2o ${TEST_DATA_DIR}/walker2_3D.g2o COPYONLY)
configure_file(test_data/walker1.tum ${TEST_DATA_DIR}/walker1.tum COPYONLY)
configure_file(test_data/walker2.tum ${TEST_DATA_DIR}/walker2.tum COPYONLY)
configure_file(test_data/test.json ${TEST_DATA_DIR}/test.json COPYONLY)

add_test(
    NAME ${TEST_BINARY}
    COMMAND ${EXECUTABLE_OUTPUT_PATH}/${TEST_BINARY}
)
