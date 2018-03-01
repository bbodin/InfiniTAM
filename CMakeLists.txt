cmake_minimum_required(VERSION 2.8)


#################################################################################################
# Find packages                                                                                 #
#################################################################################################

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(CUDA QUIET)
find_package(OpenNI QUIET)
find_package(libuvc QUIET)
find_package(RealSense QUIET)
find_package(PNG QUIET)
FIND_PACKAGE(OpenMP QUIET)



find_package(SuiteSparse REQUIRED)
find_package(Pangolin)
find_package(Eigen3   REQUIRED)

include_directories(
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)

include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.


#################################################################################################
# Source files                                                                                  #
#################################################################################################

##

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


SET(ENGINE_SOURCES
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


SET(UTILS_SOURCES
InfiniTAM/Utils/FileUtils.cpp
InfiniTAM/Utils/FileUtils.h
InfiniTAM/Utils/NVTimer.h
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




#################################################################################################
# Prepare compilation parameters                                                                #
#################################################################################################


#############################
# Generics                  #
#############################

add_definitions(-DCOMPILE_WITHOUT_OpenNI)
add_definitions(-DCOMPILE_WITHOUT_Kinect2)


include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})


IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -O3 -march=native -std=c++11 ${CFLAGS_WARN}")
ENDIF(MSVC_IDE)

if(APPLE)
  set(CUDA_HOST_COMPILER /usr/bin/clang)
endif(APPLE)

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
ENDIF()


#############################
# ITMLibCUDA               #
#############################




IF (CUDA_FOUND)

	STRING(REPLACE "_static.a" ".so" CUDA_LIBRARIES "${CUDA_LIBRARIES}")

  	SET( CUDA_PROPAGATE_HOST_FLAGS OFF ) 

	add_library(UtilsCUDA ${UTILS_SOURCES})

	IF(PNG_FOUND)
		target_compile_definitions(UtilsCUDA PRIVATE  ${PNG_DEFINITIONS})
		target_compile_definitions(UtilsCUDA PRIVATE  "-DUSE_LIBPNG")
		target_include_directories(UtilsCUDA  PRIVATE ${PNG_INCLUDE_DIRS})
		target_link_libraries(UtilsCUDA ${PNG_LIBRARIES})
	ENDIF()
	
	

	message(STATUS "LOAD FILE = ${CMAKE_SOURCE_DIR}/cmake_modules/CUDACheckCompute.cmake")
	
	if(NOT DEFINED CUDA_COMPUTE_CAPABILITY)
	  include(${CMAKE_SOURCE_DIR}/cmake_modules/CUDACheckCompute.cmake)
	endif()
	message(STATUS "CUDA_COMPUTE_CAPABILITY = ${CUDA_COMPUTE_CAPABILITY}")
	set(CUDA_ARCH ${CUDA_COMPUTE_CAPABILITY} CACHE STRING "CUDA architectures to compile for")
	message(STATUS "CUDA_ARCH = ${CUDA_ARCH}")
	string(REPLACE " " ";" CUDA_ARCH_LIST "${CUDA_ARCH}")
	message(STATUS "CUDA_ARCH_LIST = ${CUDA_ARCH_LIST}")
	foreach(a ${CUDA_ARCH_LIST})
	  set(CUDA_ARCH_OPTIONS "${CUDA_ARCH_OPTIONS} -gencode arch=compute_${a},code=compute_${a}")
	endforeach()
	
	
	
	cuda_add_library(ITMLibCUDA
	${ITMLIB_CPU_OBJECTS}
	${ITMLIB_CUDA_OBJECTS}
	${ITMLIB_COMMON_OBJECTS}
	OPTIONS "${CUDA_ARCH_OPTIONS} -O3 -std=c++11 -Xcompiler -fPIC --use_fast_math")
	#target_link_libraries(ITMLibCUDA ${CUDA_LIBRARY})
	
	target_include_directories(ITMLibCUDA  PRIVATE ${CUDA_INCLUDE_DIRS})

	add_library(EngineCUDA SHARED  ${ENGINE_SOURCES})
	
	target_link_libraries(EngineCUDA ${GLUT_LIBRARIES})
	target_link_libraries(EngineCUDA ${OPENGL_LIBRARIES})
	target_link_libraries(EngineCUDA ITMLibCUDA)
	target_link_libraries(EngineCUDA ${CUDA_LIBRARY})
	
	


ENDIF()

#############################################
# UtilsCPP, ITMLibCPP and EngineCPP   
#############################################


add_library(UtilsCPP ${UTILS_SOURCES})

target_compile_definitions(UtilsCPP PUBLIC  "-DCOMPILE_WITHOUT_OPENMP")
target_compile_definitions(UtilsCPP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")


IF(PNG_FOUND)
	target_compile_definitions(UtilsCPP PRIVATE  ${PNG_DEFINITIONS})
	target_compile_definitions(UtilsCPP PRIVATE  "-DUSE_LIBPNG")
	target_include_directories(UtilsCPP  PRIVATE ${PNG_INCLUDE_DIRS})
	target_link_libraries(UtilsCPP ${PNG_LIBRARIES})
ENDIF()



add_library(ITMLibCPP ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
target_link_libraries(ITMLibCPP UtilsCPP)

target_compile_definitions(ITMLibCPP PUBLIC  "-DCOMPILE_WITHOUT_OPENMP")
target_compile_definitions(ITMLibCPP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")

add_library(EngineCPP ${ENGINE_SOURCES})

target_link_libraries(EngineCPP ${GLUT_LIBRARIES})
target_link_libraries(EngineCPP ${OPENGL_LIBRARIES})
target_link_libraries(EngineCPP ITMLibCPP)

target_compile_definitions(EngineCPP PUBLIC  "-DCOMPILE_WITHOUT_OPENMP")
target_compile_definitions(EngineCPP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")



#############################################
# UtilsOMP, ITMLibOMP and EngineOMP   
#############################################



add_library(UtilsOMP ${UTILS_SOURCES})
SET_TARGET_PROPERTIES(UtilsOMP PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
target_compile_definitions(UtilsOMP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")
target_compile_definitions(UtilsOMP PUBLIC  "-DWITH_OPENMP")


IF(PNG_FOUND)
	target_compile_definitions(UtilsOMP PRIVATE  ${PNG_DEFINITIONS})
	target_compile_definitions(UtilsOMP PRIVATE  "-DUSE_LIBPNG")
	target_include_directories(UtilsOMP  PRIVATE ${PNG_INCLUDE_DIRS})
	target_link_libraries(UtilsOMP ${PNG_LIBRARIES})
ENDIF()



add_library(ITMLibOMP ${ITMLIB_CPU_OBJECTS} ${ITMLIB_COMMON_OBJECTS})
SET_TARGET_PROPERTIES(ITMLibOMP PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
target_link_libraries(ITMLibOMP UtilsOMP)

target_compile_definitions(ITMLibOMP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")
target_compile_definitions(ITMLibOMP PUBLIC  "-DWITH_OPENMP")

add_library(EngineOMP ${ENGINE_SOURCES})
SET_TARGET_PROPERTIES(EngineOMP PROPERTIES COMPILE_FLAGS ${OpenMP_CXX_FLAGS})

target_link_libraries(EngineOMP ${GLUT_LIBRARIES})
target_link_libraries(EngineOMP ${OPENGL_LIBRARIES})
target_link_libraries(EngineOMP ITMLibOMP)

target_compile_definitions(EngineOMP PUBLIC  "-DCOMPILE_WITHOUT_CUDA")
target_compile_definitions(EngineOMP PUBLIC  "-DWITH_OPENMP")






#################################################################################################
# SLAMbench Libraries                                                                           #
#################################################################################################

### CPP
#########

add_library(${appname}-cpp infinitam_library.cpp)
target_link_libraries(${appname}-cpp  EngineCPP )
target_include_directories(${appname}-cpp  PRIVATE "InfiniTAM/ITMLib/Engine")
target_include_directories(${appname}-cpp  PRIVATE "InfiniTAM/ORUtils")

generate_slam_library(${appname}-cpp "")

### OMP
#########

add_library(${appname}-omp infinitam_library.cpp)
target_link_libraries(${appname}-omp  EngineOMP )
target_include_directories(${appname}-omp  PRIVATE "InfiniTAM/ITMLib/Engine")
target_include_directories(${appname}-omp  PRIVATE "InfiniTAM/ORUtils")

generate_slam_library(${appname}-omp "${OpenMP_CXX_FLAGS}")


IF (CUDA_FOUND)

### CUDA
#########

add_library(${appname}-cuda infinitam_library.cpp)
target_link_libraries(${appname}-cuda  EngineCUDA )
target_include_directories(${appname}-cuda  PRIVATE "InfiniTAM/ITMLib/Engine")
target_include_directories(${appname}-cuda  PRIVATE "InfiniTAM/ORUtils")

generate_slam_library(${appname}-cuda "")

ENDIF()
    
