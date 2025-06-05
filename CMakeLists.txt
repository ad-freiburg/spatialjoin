cmake_minimum_required (VERSION 3.5)

project (spatialjoin)

IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

if(PROJECT_IS_TOP_LEVEL)
	enable_testing()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/build")

# set compiler flags, see http://stackoverflow.com/questions/7724569/debug-vs-release-in-cmake
set(CMAKE_CXX_FLAGS "-Wall -Wno-format-extra-args -Wextra -Wformat-nonliteral -Wformat-security -Wformat=2 -Wextra -Wno-implicit-fallthrough -Wno-narrowing -pedantic")
set(CMAKE_C_FLAGS "-Wall -Wno-format-extra-args -Wextra -Wformat-nonliteral -Wformat-security -Wformat=2 -Wextra -Wno-implicit-fallthrough -Wno-narrowing -pedantic")

set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS} -Og -g -DLOGLEVEL=3")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -DLOGLEVEL=2")
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -DLOGLEVEL=2")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -g -DLOGLEVEL=3")

set(CMAKE_C_FLAGS_DEBUG          "${CMAKE_C_FLAGS} -Og -g -DLOGLEVEL=3")
set(CMAKE_C_FLAGS_MINSIZEREL     "${CMAKE_C_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -DLOGLEVEL=2")
set(CMAKE_C_FLAGS_RELEASE        "${CMAKE_C_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -DLOGLEVEL=2")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS} -march=native -O3 -fno-signed-zeros -fno-trapping-math -g -DLOGLEVEL=3")

find_package(BZip2)
find_package(ZLIB)

if (NOT BZIP2_FOUND)
	add_definitions( -DSPATIALJOIN_NO_BZIP2=True )
endif()

if (NOT ZLIB_FOUND)
	add_definitions( -DSPATIALJOIN_NO_ZLIB=True )
endif()

# export compile commands to tools like clang
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Compiler-specific C++14 activation.
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
	execute_process(
		COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if ((GCC_VERSION VERSION_GREATER 4.8 OR GCC_VERSION VERSION_EQUAL 4.8))
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
	else ()
		message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.8 or greater!")
	endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else ()
	message(FATAL_ERROR "Your C++ compiler does not support C++14.")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# http://brianmilco.blogspot.de/2012/11/cmake-automatically-use-git-tags-as.html
include(GetGitRevisionDescription OPTIONAL RESULT_VARIABLE _get_git_revision)
if (_get_git_revision)
	git_get_tag(VERSION_GIT)
	get_git_is_dirty(VERSION_GIT_IS_DIRTY)
	if ("${VERSION_GIT_IS_DIRTY}" STREQUAL "")
		set(VERSION_GIT_FULL "${VERSION_GIT}")
	else()
		set(VERSION_GIT_FULL "${VERSION_GIT}-${VERSION_GIT_IS_DIRTY}")
	endif()
endif()

add_subdirectory(src)

# tests
if(CMAKE_TESTING_ENABLED)
  add_test("utilTest" utilTest)
  add_test("spatialjoinTest" spatialjoinTest)
endif()

# install target
install(
  FILES build/spatialjoin DESTINATION bin
  PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
)
