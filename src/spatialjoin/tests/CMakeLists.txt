add_executable(spatialjoinTest TestMain.cpp)
target_link_libraries(spatialjoinTest libspatialjoin util ${BZIP2_LIBRARIES} -lpthread)
