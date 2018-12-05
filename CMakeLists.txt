cmake_minimum_required(VERSION 3.9)
project(AT3DCV)

set(CMAKE_CXX_STANDARD 11)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(ORB_SLAM2_LIBRARIES /Users/Steve/Libraries/ORB_SLAM2_MAC2)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCOMPILEDWITHC11)
	message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	add_definitions(-DCOMPILEDWITHC0X)
	message(STATUS "Using flag -std=c++0x.")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 2.4.3 QUIET)
	if(NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
	endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
		${PROJECT_SOURCE_DIR}
		#${PROJECT_SOURCE_DIR}/include
		${EIGEN3_INCLUDE_DIR}
		${Pangolin_INCLUDE_DIRS}
		${ORB_SLAM2_LIBRARIES}/include
		${ORB_SLAM2_LIBRARIES}
)

#add_executable(Stereo_PennState
#		Stereo/stereo_pennstate.cc)
#target_link_libraries(Stereo_PennState
#		${OpenCV_LIBS}
#		${EIGEN3_LIBS}
#		${Pangolin_LIBRARIES}
#		${ORB_SLAM2_LIBRARIES}/Thirdparty/DBoW2/lib/libDBoW2.dylib
#		${ORB_SLAM2_LIBRARIES}/Thirdparty/g2o/lib/libg2o.dylib
#		${ORB_SLAM2_LIBRARIES}/lib/libORB_SLAM2.dylib
#)

add_executable(Mono_PennState
		Monocular/mono_PennState.cc)
target_link_libraries(Mono_PennState
		${OpenCV_LIBS}
		${EIGEN3_LIBS}
		${Pangolin_LIBRARIES}
		${ORB_SLAM2_LIBRARIES}/Thirdparty/DBoW2/lib/libDBoW2.dylib
		${ORB_SLAM2_LIBRARIES}/Thirdparty/g2o/lib/libg2o.dylib
		${ORB_SLAM2_LIBRARIES}/lib/libORB_SLAM2.dylib
)