
macro(CHECK_FILES _FILES _DIR)
set(_MISSING_FILES)
foreach(_FILE ${${_FILES}})
if(NOT EXISTS "${_FILE}")
get_filename_component(_FILE ${_FILE} NAME)
set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
endif()
endforeach()
if(_MISSING_FILES)
message(STATUS "In folder \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
set(Kinect2SDK_FOUND FALSE)
endif()
endmacro()
macro(CHECK_DIR _DIR)
if(NOT EXISTS "${${_DIR}}")
message(STATUS "Folder \"${${_DIR}}\" not found.")
set(Kinect2SDK_FOUND FALSE)
endif()
endmacro()

##################### Checking #######################
set(Kinect2SDK_ROOT $ENV{KINECT2SDK_DIR})
set(Kinect2SDK_LIB_DIR ${Kinect2SDK_ROOT}/Lib/x86)
set(Kinect2SDK_INCLUDE_DIRS ${Kinect2SDK_ROOT}/inc)
set(Kinect2SDK_LIBRARIES ${Kinect2SDK_LIB_DIR}/Kinect20.lib
${Kinect2SDK_LIB_DIR}/Kinect20.Face.lib
${Kinect2SDK_LIB_DIR}/Kinect20.Fusion.lib
${Kinect2SDK_LIB_DIR}/Kinect20.VisualGestureBuilder.lib)
mark_as_advanced(Kinect2SDK_ROOT)
mark_as_advanced(Kinect2SDK_LIB_DIR)
message(STATUS "Searching KinectSDK.")
set(Kinect2SDK_FOUND TRUE)
check_dir(Kinect2SDK_ROOT)
if(Kinect2SDK_FOUND)
check_dir(Kinect2SDK_LIB_DIR)
check_dir(Kinect2SDK_INCLUDE_DIRS)
if(Kinect2SDK_FOUND)
check_files(Kinect2SDK_LIBRARIES Kinect2SDK_LIB_DIR)
if(Kinect2SDK_FOUND)	
set(Kinect2SDK_INCLUDES ${Kinect2SDK_INCLUDE_DIRS}/Kinect.Face.h
${Kinect2SDK_INCLUDE_DIRS}/Kinect.h
${Kinect2SDK_INCLUDE_DIRS}/Kinect.INPC.h
${Kinect2SDK_INCLUDE_DIRS}/Kinect.VisualGestureBuilder.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionApi.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionBase.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionCameraPoseFinder.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionColorVolume.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionDepthProcessor.h
${Kinect2SDK_INCLUDE_DIRS}/NuiKinectFusionVolume.h)
mark_as_advanced(Kinect2SDK_INCLUDES)
check_files(Kinect2SDK_INCLUDES Kinect2SDK_INCLUDE_DIRS)
endif()
endif()
endif()
message(STATUS "Kinect2SDK_FOUND: ${Kinect2SDK_FOUND}.")
