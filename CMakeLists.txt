cmake_minimum_required( VERSION 3.5.1 )
project( rgbd_odometry )

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS filesystem system REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )

# OpenCV Visual Odometry implementation
add_executable( ocv_track src/ocv_track.cpp )
target_link_libraries( ocv_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )

# Fovis Visual Odometry implementation
find_package( Eigen3 3.3 REQUIRED )
include_directories( ${CMAKE_SOURCE_DIR}/third-party/fovis/build/include )
link_directories( ${CMAKE_SOURCE_DIR}/third-party/fovis/build/lib )
add_executable( fovis_track ${CMAKE_SOURCE_DIR}/src/fovis_track.cpp )
target_link_libraries( fovis_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} fovis Eigen3::Eigen )

# DVO implementation
find_package( Eigen3 3.3 REQUIRED )
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/third-party/dvo/include )
link_directories( ${CMAKE_SOURCE_DIR}/third-party/dvo/build )
add_executable( dvo_track ${CMAKE_SOURCE_DIR}/src/dvo_track.cpp )
target_link_libraries( dvo_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} tbb dvo_core Eigen3::Eigen )

# vors implementation (just compile the rust binary)
add_custom_target( vors_track
	ALL cargo install --force --root ${CMAKE_BINARY_DIR} --path .
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/third-party/visual-odometry-rs
)
