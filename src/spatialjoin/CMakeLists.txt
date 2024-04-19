file(GLOB_RECURSE SPATIALJOIN_SRC *.cpp)
list(REMOVE_ITEM SPATIALJOIN_SRC TestMain.cpp)

set(spatialjoin_main SpatialJoinMain.cpp)

list(REMOVE_ITEM SPATIALJOIN_SRC ${spatialjoin_main})

include_directories(
	${SPATIALJOIN_INCLUDE_DIR}
	${ZLIB_INCLUDE_DIRS}
)

add_executable(spatialjoin ${spatialjoin_main})
add_library(spatialjoin_dep ${SPATIALJOIN_SRC})

target_link_libraries(spatialjoin spatialjoin_dep util ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES} -lpthread)

add_subdirectory(tests)
