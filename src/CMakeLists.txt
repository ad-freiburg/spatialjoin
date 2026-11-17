set(SPATIALJOIN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(util)
add_subdirectory(spatialjoin)

option(SPATIALJOIN_BUILD_EXAMPLES "Build the example programs" ${PROJECT_IS_TOP_LEVEL})
if (SPATIALJOIN_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()
