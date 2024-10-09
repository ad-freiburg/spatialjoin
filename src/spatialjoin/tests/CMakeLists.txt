add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest spatialjoin-dev util ${BZIP2_LIBRARIES} -lpthread)
