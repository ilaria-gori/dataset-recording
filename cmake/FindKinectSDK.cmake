
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
set(KinectSDK_FOUND FALSE)
endif()
endmacro()
macro(CHECK_DIR _DIR)
if(NOT EXISTS "${${_DIR}}")
message(STATUS "Folder \"${${_DIR}}\" not found.")
set(KinectSDK_FOUND FALSE)
endif()
endmacro()

##################### Checking #######################
set(KinectSDK_ROOT $ENV{KINECTSDK_DIR})
set(KinectSDK_LIB_DIR ${KinectSDK_ROOT}/Lib/x86)
set(KinectSDK_INCLUDE_DIRS ${KinectSDK_ROOT}/inc)
set(KinectSDK_LIBRARIES ${KinectSDK_LIB_DIR}/Kinect20.lib
${KinectSDK_LIB_DIR}/Kinect20.Face.lib
${KinectSDK_LIB_DIR}/Kinect20.Fusion.lib
${KinectSDK_LIB_DIR}/Kinect20.VisualGestureBuilder.lib)
mark_as_advanced(KinectSDK_ROOT)
mark_as_advanced(KinectSDK_LIB_DIR)
message(STATUS "Searching KinectSDK.")
set(KinectSDK_FOUND TRUE)
check_dir(KinectSDK_ROOT)
if(KinectSDK_FOUND)
check_dir(KinectSDK_LIB_DIR)
check_dir(KinectSDK_INCLUDE_DIRS)
if(KinectSDK_FOUND)
check_files(KinectSDK_LIBRARIES KinectSDK_LIB_DIR)
if(KinectSDK_FOUND)	
set(KinectSDK_INCLUDES ${KinectSDK_INCLUDE_DIRS}/Kinect.Face.h
${KinectSDK_INCLUDE_DIRS}/Kinect.h
${KinectSDK_INCLUDE_DIRS}/Kinect.INPC.h
${KinectSDK_INCLUDE_DIRS}/Kinect.VisualGestureBuilder.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionApi.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionBase.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionCameraPoseFinder.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionColorVolume.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionDepthProcessor.h
${KinectSDK_INCLUDE_DIRS}/NuiKinectFusionVolume.h)
mark_as_advanced(KinectSDK_INCLUDES)
check_files(KinectSDK_INCLUDES KinectSDK_INCLUDE_DIRS)
endif()
endif()
endif()
message(STATUS "KinectSDK_FOUND: ${KinectSDK_FOUND}.")
