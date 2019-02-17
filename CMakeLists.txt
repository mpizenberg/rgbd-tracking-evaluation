cmake_minimum_required( VERSION 3.10 )
project( rgbd_odometry )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS filesystem REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3")

# OpenCV Visual Odometry implementation
add_executable( ocv_track src/ocv_track.cpp )
target_link_libraries( ocv_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} )

# Fovis Visual Odometry implementation
find_package(Eigen3 3.3 REQUIRED)
include_directories( /home/matthieu/git/phd/slam/fovis/build/include )
link_directories( /home/matthieu/git/phd/slam/fovis/build/lib )
add_executable( fovis_track src/fovis_track.cpp )
target_link_libraries( fovis_track ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} fovis Eigen3::Eigen )
