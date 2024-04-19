add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest spatialjoin_dep util ${BZIP2_LIBRARIES} -lpthread)
