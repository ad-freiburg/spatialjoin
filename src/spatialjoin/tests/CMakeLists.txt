add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest spatialjoin-dev pb_util ${BZIP2_LIBRARIES} -lpthread -lgeos_c)
