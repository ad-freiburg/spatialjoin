add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest spatialjoin-dev pb_util pb_util_geo ${BZIP2_LIBRARIES} -lpthread)
