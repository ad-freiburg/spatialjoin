add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest spatialjoin-dev pb_util pb_util_geo ${BZIP2_LIBRARIES} -lpthread)
target_compile_definitions(spatialjoinTest PRIVATE
  TEST_DATASET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/datasets")

if(PROJECT_IS_TOP_LEVEL AND CMAKE_TESTING_ENABLED)
  add_test("spatialjoinTest" spatialjoinTest)
endif()
