SET(PROJECT_NAME SLAM_IT)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#set(CMAKE_VERBOSE_MAKEFILE ON)


set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(DSO_PATH ~/repos/SLAM_IT/vidso/)

# required libraries
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_package(OpenCV REQUIRED)
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)


# flags
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
)


# Sources files
set(slamit_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/coder/base64.cpp
)



include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
) 




add_library(helper ${slamit_SOURCE_FILES})


# build base64 encoder to act as a tester
message("--- compiling Tester.")
add_executable(encode ${PROJECT_SOURCE_DIR}/src/coder/tester.cpp )
target_link_libraries(encode helper)

# build main executable 
message("--- compiling SLAM_IT.")
add_executable(slam_it ${PROJECT_SOURCE_DIR}/src/main.cpp )
target_link_libraries(slam_it helper boost_thread boost_system ${DSO_LIBRARY} ${OpenCV_LIBS})

