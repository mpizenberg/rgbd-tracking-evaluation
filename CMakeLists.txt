cmake_minimum_required( VERSION 3.10 )
project( rgbd_odometry )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS filesystem REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# OpenCV Visual Odometry implementation
add_executable( ocv_track src/ocv_track.cpp )
target_link_libraries( ocv_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} )

# Fovis Visual Odometry implementation
find_package(Eigen3 3.3 REQUIRED)
include_directories( third-party/fovis/build/include )
link_directories( third-party/fovis/build/lib )
add_executable( fovis_track src/fovis_track.cpp )
target_link_libraries( fovis_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} fovis Eigen3::Eigen )
