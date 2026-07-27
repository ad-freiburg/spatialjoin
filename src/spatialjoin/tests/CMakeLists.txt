add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest PRIVATE spatialjoin-dev pb_util pb_util_geo -lpthread)
if (BZIP2_FOUND)
	target_link_libraries(spatialjoinTest PRIVATE ${BZIP2_LIBRARIES} )
endif(BZIP2_FOUND)
target_compile_definitions(spatialjoinTest PRIVATE
  TEST_DATASET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/datasets")

if(PROJECT_IS_TOP_LEVEL AND CMAKE_TESTING_ENABLED)
	add_test(NAME "spatialjoinTest" COMMAND spatialjoinTest)
endif()
