cmake_minimum_required(VERSION 2.8)

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})

OPTION(WITH_OPENMP "Enable OpenMP support?" OFF)
IF(WITH_OPENMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
  ADD_DEFINITIONS(-DWITH_OPENMP)
ENDIF()


OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN}")
  #set(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()

SET(ORUTILS_HEADERS
InfiniTAM/ORUtils/Vector.h
InfiniTAM/ORUtils/Matrix.h
InfiniTAM/ORUtils/Cholesky.h
InfiniTAM/ORUtils/MathUtils.h
InfiniTAM/ORUtils/Image.h
InfiniTAM/ORUtils/CUDADefines.h
InfiniTAM/ORUtils/LexicalCast.h
InfiniTAM/ORUtils/MemoryBlock.h
InfiniTAM/ORUtils/MemoryBlockPersister.h
InfiniTAM/ORUtils/PlatformIndependence.h
)

set(ORUTILS_OBJECTS
InfiniTAM/ORUtils/Dummy.cpp
${ORUTILS_HEADERS}
)

SOURCE_GROUP("" FILES ${ORUTILS_HEADERS})
add_library(ORUtils ${ORUTILS_OBJECTS})

#############################
# Specify the project files #
#############################

##
SET(ITMLIB_TOPLEVEL_HEADERS
InfiniTAM/ITMLib/ITMLib.h
)

##
SET(ITMLIB_ENGINE_SOURCES
InfiniTAM/ITMLib/Engine/ITMColorTracker.cpp
InfiniTAM/ITMLib/Engine/ITMDenseMapper.cpp
InfiniTAM/ITMLib/Engine/ITMDepthTracker.cpp
InfiniTAM/ITMLib/Engine/ITMWeightedICPTracker.cpp
InfiniTAM/ITMLib/Engine/ITMIMUTracker.cpp
InfiniTAM/ITMLib/Engine/ITMMainEngine.cpp
InfiniTAM/ITMLib/Engine/ITMRenTracker.cpp
InfiniTAM/ITMLib/Engine/ITMTrackerFactory.cpp
InfiniTAM/ITMLib/Engine/ITMTrackingController.cpp
InfiniTAM/ITMLib/Engine/ITMVisualisationEngine.cpp
)

SET(ITMLIB_ENGINE_HEADERS
InfiniTAM/ITMLib/Engine/ITMColorTracker.h
InfiniTAM/ITMLib/Engine/ITMCompositeTracker.h
InfiniTAM/ITMLib/Engine/ITMDenseMapper.h
InfiniTAM/ITMLib/Engine/ITMDepthTracker.h
InfiniTAM/ITMLib/Engine/ITMWeightedICPTracker.h
InfiniTAM/ITMLib/Engine/ITMIMUCalibrator.h
InfiniTAM/ITMLib/Engine/ITMIMUTracker.h
InfiniTAM/ITMLib/Engine/ITMLowLevelEngine.h
InfiniTAM/ITMLib/Engine/ITMMainEngine.h
InfiniTAM/ITMLib/Engine/ITMRenTracker.h
InfiniTAM/ITMLib/Engine/ITMSceneReconstructionEngine.h
InfiniTAM/ITMLib/Engine/ITMSwappingEngine.h
InfiniTAM/ITMLib/Engine/ITMTracker.h
InfiniTAM/ITMLib/Engine/ITMTrackerFactory.h
InfiniTAM/ITMLib/Engine/ITMTrackingController.h
InfiniTAM/ITMLib/Engine/ITMViewBuilder.h
InfiniTAM/ITMLib/Engine/ITMVisualisationEngine.h
InfiniTAM/ITMLib/Engine/ITMMeshingEngine.h
)

##
set(ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMColorTracker.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMDepthTracker.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMWeightedICPTracker.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMLowLevelEngine.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMPixelUtils.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMRenTracker.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMRepresentationAccess.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMSceneReconstructionEngine.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMSwappingEngine.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMViewBuilder.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMVisualisationEngine.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMPixelUtils.h
InfiniTAM/ITMLib/Engine/DeviceAgnostic/ITMMeshingEngine.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.cpp
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.cpp
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMColorTracker_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMDepthTracker_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMWeightedICPTracker_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMLowLevelEngine_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMRenTracker_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMSceneReconstructionEngine_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMSwappingEngine_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMViewBuilder_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMVisualisationEngine_CPU.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CPU/ITMMeshingEngine_CPU.h
)

##
set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.cu
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMMeshingEngine_CUDA.cu
)

set(ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMColorTracker_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMCUDAUtils.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMDepthTracker_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMWeightedICPTracker_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMLowLevelEngine_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMRenTracker_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMSceneReconstructionEngine_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMSwappingEngine_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMViewBuilder_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMVisualisationEngine_CUDA.h
InfiniTAM/ITMLib/Engine/DeviceSpecific/CUDA/ITMMeshingEngine_CUDA.h
)

##
set(ITMLIB_OBJECTS_SOURCES
InfiniTAM/ITMLib/Objects/ITMPose.cpp
)

set(ITMLIB_OBJECTS_HEADERS
InfiniTAM/ITMLib/Objects/ITMDisparityCalib.h
InfiniTAM/ITMLib/Objects/ITMExtrinsics.h
InfiniTAM/ITMLib/Objects/ITMGlobalCache.h
InfiniTAM/ITMLib/Objects/ITMImageHierarchy.h
InfiniTAM/ITMLib/Objects/ITMIntrinsics.h
InfiniTAM/ITMLib/Objects/ITMLocalVBA.h
InfiniTAM/ITMLib/Objects/ITMPlainVoxelArray.h
InfiniTAM/ITMLib/Objects/ITMPointCloud.h
InfiniTAM/ITMLib/Objects/ITMPose.h
InfiniTAM/ITMLib/Objects/ITMRGBDCalib.h
InfiniTAM/ITMLib/Objects/ITMScene.h
InfiniTAM/ITMLib/Objects/ITMSceneHierarchyLevel.h
InfiniTAM/ITMLib/Objects/ITMSceneParams.h
InfiniTAM/ITMLib/Objects/ITMTemplatedHierarchyLevel.h
InfiniTAM/ITMLib/Objects/ITMTrackingState.h
InfiniTAM/ITMLib/Objects/ITMView.h
InfiniTAM/ITMLib/Objects/ITMViewIMU.h
InfiniTAM/ITMLib/Objects/ITMViewHierarchyLevel.h
InfiniTAM/ITMLib/Objects/ITMRenderState.h
InfiniTAM/ITMLib/Objects/ITMRenderState_VH.h
InfiniTAM/ITMLib/Objects/ITMVoxelBlockHash.h
InfiniTAM/ITMLib/Objects/ITMIMUMeasurement.h
InfiniTAM/ITMLib/Objects/ITMMesh.h
)

##
set(ITMLIB_UTILS_SOURCES
InfiniTAM/ITMLib/Utils/ITMCalibIO.cpp
InfiniTAM/ITMLib/Utils/ITMLibSettings.cpp
)

set(ITMLIB_UTILS_HEADERS
InfiniTAM/ITMLib/Utils/ITMCalibIO.h
InfiniTAM/ITMLib/Utils/ITMLibDefines.h
InfiniTAM/ITMLib/Utils/ITMLibSettings.h
InfiniTAM/ITMLib/Utils/ITMMath.h
)

#################################################################
# Collect the project files into common, CPU-only and CUDA-only #
#################################################################

set(ITMLIB_COMMON_OBJECTS
${ITMLIB_TOPLEVEL_HEADERS}
${ITMLIB_ENGINE_SOURCES}
${ITMLIB_ENGINE_HEADERS}
${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS}
${ITMLIB_OBJECTS_SOURCES}
${ITMLIB_OBJECTS_HEADERS}
${ITMLIB_UTILS_SOURCES}
${ITMLIB_UTILS_HEADERS}
)

set(ITMLIB_CPU_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS}
)

set(ITMLIB_CUDA_OBJECTS
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES}
${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS}
)

#############################
# Specify the source groups #
#############################

SOURCE_GROUP("" FILES ${ITMLIB_TOPLEVEL_HEADERS})
SOURCE_GROUP(Engine FILES ${ITMLIB_ENGINE_SOURCES} ${ITMLIB_ENGINE_HEADERS})
SOURCE_GROUP(Engine\\DeviceAgnostic FILES ${ITMLIB_ENGINE_DEVICEAGNOSTIC_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CPU FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CPU_HEADERS})
SOURCE_GROUP(Engine\\DeviceSpecific\\CUDA FILES ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_SOURCES} ${ITMLIB_ENGINE_DEVICESPECIFIC_CUDA_HEADERS})
SOURCE_GROUP(Objects FILES ${ITMLIB_OBJECTS_SOURCES} ${ITMLIB_OBJECTS_HEADERS})
SOURCE_GROUP(Utils FILES ${ITMLIB_UTILS_SOURCES} ${ITMLIB_UTILS_HEADERS})

##############################################################
# Specify the include directories, target and link libraries #
##############################################################

if(WITH_CUDA)
  if(NOT DEFINED CUDA_COMPUTE_CAPABILITY)
    include(${PROJECT_SOURCE_DIR}/cmake/CUDACheckCompute.cmake)
  endif()
  message(STATUS "CUDA_COMPUTE_CAPABILITY = ${CUDA_COMPUTE_CAPABILITY}")
  set(CUDA_ARCH ${CUDA_COMPUTE_CAPABILITY} CACHE STRING "CUDA architectures to compile for")
  message(STATUS "CUDA_ARCH = ${CUDA_ARCH}")
  string(REPLACE " " ";" CUDA_ARCH_LIST "${CUDA_ARCH}")
  message(STATUS "CUDA_ARCH_LIST = ${CUDA_ARCH_LIST}")
  foreach(a ${CUDA_ARCH_LIST})
    set(CUDA_ARCH_OPTIONS "${CUDA_ARCH_OPTIONS} -gencode arch=compute_${a},code=compute_${a}")
  endforeach()
  include_directories(${CUDA_INCLUDE_DIRS})
  cuda_add_library(ITMLib
	${ITMLIB_CPU_OBJECTS}
	${ITMLIB_CUDA_OBJECTS}
	${ITMLIB_COMMON_OBJECTS}
	OPTIONS "${CUDA_ARCH_OPTIONS} --use_fast_math")
  target_link_libraries(ITMLib ${CUDA_LIBRARY})
else()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
  add_library(ITMLib ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
endif()

target_link_libraries(ITMLib Utils)


IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(PNG_FOUND)
  include_directories(${PNG_INCLUDE_DIRS})
  add_definitions(${PNG_DEFINITIONS})
  add_definitions(-DUSE_LIBPNG)
ENDIF()

add_library(Utils
InfiniTAM/Utils/FileUtils.cpp
InfiniTAM/Utils/FileUtils.h
InfiniTAM/Utils/NVTimer.h
)

IF(PNG_FOUND)
  target_link_libraries(Utils ${PNG_LIBRARIES})
ENDIF()



include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

IF(OPENNI_FOUND)
  include_directories(${OpenNI_INCLUDE_DIR})
ELSE(OPENNI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_OpenNI)
ENDIF(OPENNI_FOUND)

IF(MSKINECTAPI_FOUND)
  include_directories(${MSKINECTAPI_INCLUDE_DIR})
ELSE(MSKINECTAPI_FOUND)
  add_definitions(-DCOMPILE_WITHOUT_Kinect2)
ENDIF(MSKINECTAPI_FOUND)

IF(DEFINED libuvc_INCLUDE_DIRS)
  include_directories(${libuvc_INCLUDE_DIRS})
  add_definitions(-DCOMPILE_WITH_LibUVC)
ENDIF()

IF(WITH_REALSENSE)
  include_directories(${RealSense_INCLUDE_DIR})
  add_definitions(-DCOMPILE_WITH_RealSense)
  SET_PROPERTY(SOURCE InfiniTAM/Engine/RealSenseEngine.cpp PROPERTY COMPILE_FLAGS -std=c++11)
ENDIF()

add_library(Engine
InfiniTAM/Engine/ImageSourceEngine.cpp
InfiniTAM/Engine/ImageSourceEngine.h
InfiniTAM/Engine/IMUSourceEngine.cpp
InfiniTAM/Engine/IMUSourceEngine.h
InfiniTAM/Engine/Kinect2Engine.cpp
InfiniTAM/Engine/Kinect2Engine.h
InfiniTAM/Engine/OpenNIEngine.cpp
InfiniTAM/Engine/OpenNIEngine.h
InfiniTAM/Engine/LibUVCEngine.cpp
InfiniTAM/Engine/LibUVCEngine.h
InfiniTAM/Engine/UIEngine.cpp
InfiniTAM/Engine/UIEngine.h
InfiniTAM/Engine/CLIEngine.cpp
InfiniTAM/Engine/CLIEngine.h
InfiniTAM/Engine/RealSenseEngine.cpp
InfiniTAM/Engine/RealSenseEngine.h
)

target_link_libraries(Engine ${GLUT_LIBRARIES})
target_link_libraries(Engine ${OPENGL_LIBRARIES})
target_link_libraries(Engine ITMLib)
IF(WITH_CUDA)
  target_link_libraries(Engine ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(Engine ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(Engine ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(Engine ${RealSense_LIBRARY})
ENDIF()




IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()

add_executable(InfiniTAM_cli InfiniTAM/InfiniTAM_cli.cpp)
target_link_libraries(InfiniTAM_cli Engine)
target_link_libraries(InfiniTAM_cli Utils)

add_executable(InfiniTAM InfiniTAM/InfiniTAM.cpp)
target_link_libraries(InfiniTAM Engine)
target_link_libraries(InfiniTAM Utils)



include_directories("InfiniTAM/ITMLib/Engine")

    add_library(${appname}-original  
InfiniTAM/Engine/ImageSourceEngine.cpp
InfiniTAM/Engine/ImageSourceEngine.h
InfiniTAM/Engine/IMUSourceEngine.cpp
InfiniTAM/Engine/IMUSourceEngine.h
InfiniTAM/Engine/Kinect2Engine.cpp
InfiniTAM/Engine/Kinect2Engine.h
InfiniTAM/Engine/OpenNIEngine.cpp
InfiniTAM/Engine/OpenNIEngine.h
InfiniTAM/Engine/LibUVCEngine.cpp
InfiniTAM/Engine/LibUVCEngine.h
InfiniTAM/Engine/UIEngine.cpp
InfiniTAM/Engine/UIEngine.h
InfiniTAM/Engine/CLIEngine.cpp
InfiniTAM/Engine/CLIEngine.h
InfiniTAM/Engine/RealSenseEngine.cpp
InfiniTAM/Engine/RealSenseEngine.h
    )
    
target_link_libraries(${appname}-original   ${GLUT_LIBRARIES})
target_link_libraries(${appname}-original   ${OPENGL_LIBRARIES})
target_link_libraries(${appname}-original   ITMLib)
IF(WITH_CUDA)
  target_link_libraries(${appname}-original   ${CUDA_LIBRARY})
ENDIF()
IF(OPENNI_FOUND)
  target_link_libraries(${appname}-original   ${OpenNI_LIBRARY})
ENDIF(OPENNI_FOUND)
IF(DEFINED libuvc_LIBRARIES)
  target_link_libraries(${appname}-original   ${libuvc_LIBRARIES})
ENDIF()
IF(WITH_REALSENSE)
  TARGET_LINK_LIBRARIES(${appname}-original   ${RealSense_LIBRARY})

ENDIF()
      TARGET_LINK_LIBRARIES(${appname}-original   pthread)
    
add_slambench_binary(${appname}  benchmark    original "" "-D_GLIBCXX_USE_CXX11_ABI=1" "")
add_slambench_binary(${appname}  main         original "" "-D_GLIBCXX_USE_CXX11_ABI=1" "")
