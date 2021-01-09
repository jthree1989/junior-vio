cmake_minimum_required(VERSION 3.1)
project(Junior_VIO CXX)

# Download automatically, you can also just copy the conan.cmake file
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
   message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
   file(DOWNLOAD "https://github.com/conan-io/cmake-conan/raw/v0.15/conan.cmake"
                 "${CMAKE_BINARY_DIR}/conan.cmake" 
                 TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

set(CMAKE_CXX_FLAGS "-Wall -std=c++17")

option(OPTION_BUILD_TESTS     		"Build tests."                OFF)
option(OPTION_BUILD_DOCS      		"Build documentation."        OFF)
option(OPTION_BUILD_TOOLS     		"Build tools."                OFF)

option(OPTION_BUILD_WITH_OPENCV 	"Build with opencv"			  ON)

set(CONAN_PROFILE default)

conan_cmake_run(REQUIRES eigen/3.3.7@conan/stable
			    REQUIRES ceres-solver/1.14.0
				REQUIRES spdlog/1.8.1 OPTIONS spdlog:header_only=True
			    REQUIRES toml11/3.6.0
				REQUIRES Catch2/2.11.1@catchorg/stable
				REQUIRES fmt/7.0.3 OPTIONS fmt:header_only=True
			    REQUIRES sophus/1.0.0
				REQUIRES boost/1.74.0 OPTIONS boost:header_only=True
				REQUIRES opencv/4.2.0@conan/stable
			    BASIC_SETUP CMAKE_TARGETS
			    SETTINGS build_type=Release 
				BUILD missing)

conan_global_flags()

add_subdirectory(src)

if(OPTION_BUILD_TOOLS)
	add_subdirectory(tools)
endif()

if(OPTION_BUILD_TESTS)
	add_subdirectory(test)
endif()

if(OPTION_BUILD_DOCS)
	add_subdirectory(docs)
endif()