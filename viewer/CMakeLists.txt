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
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
#   "${SSE_FLAGS} -O3 -g -std=c++0x -fno-omit-frame-pointer"
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


# Sources files
#set(view_SOURCE_FILES
#  ${PROJECT_SOURCE_DIR}/src/
#)






add_executable(view_cloud ${PROJECT_SOURCE_DIR}/src/main_view.cpp )
target_link_libraries(view_cloud ${PCL_LIBRARIES})

add_executable(view_example ${PROJECT_SOURCE_DIR}/src/main2.cpp )
target_link_libraries(view_example ${PCL_LIBRARIES})

add_executable(view_example3 ${PROJECT_SOURCE_DIR}/src/main3.cpp )
target_link_libraries(view_example3 ${PCL_LIBRARIES})

add_executable (statistical_removal ${PROJECT_SOURCE_DIR}/src/outlier3.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (triangle3 ${PROJECT_SOURCE_DIR}/src/triangle3.cpp)
target_link_libraries (triangle3 ${PCL_LIBRARIES})


add_executable (mels3 ${PROJECT_SOURCE_DIR}/src/mels3.cpp)
target_link_libraries (mels3 ${PCL_LIBRARIES})

add_executable (mls3 ${PROJECT_SOURCE_DIR}/src/mls3.cpp)
target_link_libraries (mls3 ${PCL_LIBRARIES})

