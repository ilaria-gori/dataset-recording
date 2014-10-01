cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME dataset-recording)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(KinectSDK REQUIRED)

file(GLOB sources src/*.cpp src/hog/*.cpp)
file(GLOB headers include/*.h include/hog/*.h)

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${KinectSDK_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${KinectSDK_LIB_DIR})

add_executable(${PROJECTNAME} ${sources} ${headers})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${KinectSDK_LIBRARIES})
