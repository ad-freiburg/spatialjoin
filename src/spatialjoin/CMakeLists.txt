file(GLOB_RECURSE SPATIALJOIN_SRC *.cpp)
list(REMOVE_ITEM SPATIALJOIN_SRC TestMain.cpp)

set(spatialjoin_main SpatialJoinMain.cpp)

list(REMOVE_ITEM SPATIALJOIN_SRC ${spatialjoin_main})

add_executable(spatialjoin ${spatialjoin_main})
add_library(spatialjoin-dev ${SPATIALJOIN_SRC})

target_include_directories(spatialjoin PUBLIC
	$<BUILD_INTERFACE:${SPATIALJOIN_INCLUDE_DIR}>
	${ZLIB_INCLUDE_DIRS}
	${BZIP2_INCLUDE_DIR}
)

target_include_directories(spatialjoin-dev PUBLIC
	$<BUILD_INTERFACE:${SPATIALJOIN_INCLUDE_DIR}>
	${ZLIB_INCLUDE_DIRS}
	${BZIP2_INCLUDE_DIR}
)

if (BZIP2_FOUND)
	add_definitions( -DBZIP2_FOUND=${BZIP2_FOUND} )
endif()

if (ZLIB_FOUND)
	add_definitions( -DZLIB_FOUND=${ZLIB_FOUND} )
endif()

target_link_libraries(spatialjoin spatialjoin-dev pb_util ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} -lpthread)

add_subdirectory(tests)
