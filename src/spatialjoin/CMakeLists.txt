file(GLOB_RECURSE SPATIALJOIN_SRC *.cpp)
list(REMOVE_ITEM SPATIALJOIN_SRC TestMain.cpp)

if(NOT CMAKE_TESTING_ENABLED)
	list(FILTER util_SRC EXCLUDE REGEX ".*/tests/.*")
endif()

set(spatialjoin_main SpatialJoinMain.cpp)

list(REMOVE_ITEM SPATIALJOIN_SRC ${spatialjoin_main})

add_executable(spatialjoin ${spatialjoin_main})
add_library(spatialjoin-dev ${SPATIALJOIN_SRC})

target_include_directories(spatialjoin PUBLIC
	$<BUILD_INTERFACE:${SPATIALJOIN_INCLUDE_DIR}>
)

target_include_directories(spatialjoin-dev PUBLIC
	$<BUILD_INTERFACE:${SPATIALJOIN_INCLUDE_DIR}>
)

target_link_libraries(spatialjoin spatialjoin-dev pb_util -lpthread)

if (BZIP2_FOUND)
	target_include_directories(spatialjoin PUBLIC ${BZIP2_INCLUDE_DIR} )
	target_include_directories(spatialjoin-dev PUBLIC ${BZIP2_INCLUDE_DIR} )
	target_link_libraries(spatialjoin ${BZIP2_LIBRARIES} )
endif(BZIP2_FOUND)

if (ZLIB_FOUND)
	target_include_directories(spatialjoin INTERFACE ${ZLIB_INCLUDE_DIRS} )
	target_include_directories(spatialjoin-dev INTERFACE ${ZLIB_INCLUDE_DIRS} )
	target_link_libraries(spatialjoin ${ZLIB_LIBRARIES} )
endif(ZLIB_FOUND)


if(CMAKE_TESTING_ENABLED)
    add_subdirectory(tests)
endif()
