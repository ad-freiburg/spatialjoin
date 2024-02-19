set(SPATIALJOIN_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_BINARY_DIR})

include_directories(
	${SPATIALJOIN_INCLUDE_DIR}
)

include_directories( SYSTEM ${BZIP2_INCLUDE_DIR} )

add_subdirectory(util)
add_subdirectory(spatialjoin)
