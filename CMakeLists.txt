cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME dataset-recording)
project(${PROJECTNAME})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Kinect2SDK)
find_package(KinectSDK)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

set(headers include/cameraDriver.h
	include/dirent.h
	include/driver.h)
set(sources src/cameraDriver.cpp
	src/main.cpp)
	
if (Kinect2SDK_FOUND)

	list(APPEND headers include/kinect2Driver.h)
	list(APPEND sources src/kinect2Driver.cpp)

	include_directories(${Kinect2SDK_INCLUDE_DIRS})
	link_directories(${Kinect2SDK_LIB_DIR})
endif()	

if (KinectSDK_FOUND)

	list(APPEND headers include/kinectDriver.h)
	list(APPEND sources src/kinectDriver.cpp)

	include_directories(${KinectSDK_INCLUDE_DIRS})
	link_directories(${KinectSDK_LIB_DIR})
endif()

source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${sources} ${headers})

if (Kinect2SDK_FOUND AND KinectSDK_FOUND)
	target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${Kinect2SDK_LIBRARIES} ${KinectSDK_LIBRARIES})
elseif (Kinect2SDK_FOUND AND (NOT KinectSDK_FOUND))
	target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${Kinect2SDK_LIBRARIES})
elseif ((NOT Kinect2SDK_FOUND) AND KinectSDK_FOUND)
	target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${KinectSDK_LIBRARIES})
elseif ((NOT Kinect2SDK_FOUND) AND (NOT KinectSDK_FOUND))
	target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES})
endif()
