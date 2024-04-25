file(GLOB_RECURSE SPATIALJOIN_SRC *.cpp)

set(spatialjoin_main SpatialJoinMain.cpp)

list(REMOVE_ITEM SPATIALJOIN_SRC ${spatialjoin_main})

include_directories(
	${SPATIALJOIN_INCLUDE_DIR}
)

add_executable(spatialjoin ${spatialjoin_main}
		WktParser.cpp
		WktParser.h)
add_library(spatialjoin_dep ${SPATIALJOIN_SRC})
target_link_libraries(spatialjoin_dep absl::strings)

target_link_libraries(spatialjoin spatialjoin_dep util ${BZIP2_LIBRARIES} -lpthread)
