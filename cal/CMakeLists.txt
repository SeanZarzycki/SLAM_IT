SET(PROJECT_NAME VIEWER)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#set(CMAKE_VERBOSE_MAKEFILE ON)


set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   #"${SSE_FLAGS} -O3 -g -std=c++0x -march=native -Wall"
   "${SSE_FLAGS} -O0 -g -std=c++0x -march=native -Wall" # no optimizations
)


find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
) 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(fuselib ${PROJECT_SOURCE_DIR}/src/edge_det.cpp ${PROJECT_SOURCE_DIR}/src/Fuse.cpp ${PROJECT_SOURCE_DIR}/src/Fuse_help.cpp)


add_executable(data_convert ${PROJECT_SOURCE_DIR}/src/Data_Converter.cpp )
target_link_libraries(data_convert ${PCL_LIBRARIES})

add_executable(view_example3 ${PROJECT_SOURCE_DIR}/src/main3.cpp )
target_link_libraries(view_example3 ${PCL_LIBRARIES})

add_executable (statistical_removal ${PROJECT_SOURCE_DIR}/src/outlier3.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (fuse ${PROJECT_SOURCE_DIR}/src/iter_fuse.cpp)
target_link_libraries (fuse fuselib ${PCL_LIBRARIES})

add_executable (calib ${PROJECT_SOURCE_DIR}/src/camera_calibration.cpp)
target_link_libraries (calib ${OpenCV_LIBS})

add_executable(demo ${PROJECT_SOURCE_DIR}/src/Demo.cpp )
target_link_libraries(demo ${PCL_LIBRARIES})

add_executable(plane_maker ${PROJECT_SOURCE_DIR}/src/plane_maker.cpp )
target_link_libraries(plane_maker ${PCL_LIBRARIES})

add_executable(slam_converter ${PROJECT_SOURCE_DIR}/src/slam_converter.cpp )
target_link_libraries(slam_converter ${PCL_LIBRARIES})
