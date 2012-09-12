PROJECT(PhotoconsistencyVisualOdometry)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# Set the output directory for the build executables
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# Set the cmake find libraries directory
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake) 

#Add here your project dependencies
	# OpenCV library
	FIND_PACKAGE(OpenCV REQUIRED )

	# PCL library
	FIND_PACKAGE(PCL REQUIRED)
	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
	ADD_DEFINITIONS(${PCL_DEFINITIONS})
	
	# MRPT library
	FIND_PACKAGE(MRPT REQUIRED hwdrivers maps graphslam) 

	# Boost library
	FIND_PACKAGE( Boost 1.47.0 REQUIRED )
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} ) 

	# OpenMP
	FIND_PACKAGE(OpenMP)
	IF(OPENMP_FOUND)
	    MESSAGE("-- Found OpenMP.")
	    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} 						${OpenMP_EXE_LINKER_FLAGS}")
	ELSE(OPENMP_FOUND)
	    MESSAGE("-- Can't find OpenMP. Continuing without it.")
	ENDIF(OPENMP_FOUND)

	# Google Flags
	OPTION(GFLAGS
	       "Enable Google Flags."
	       ON)

	IF (GFLAGS)
	  MESSAGE("-- Check for Google Flags")
	  FIND_LIBRARY(GFLAGS_LIB NAMES gflags PATHS ${SEARCH_LIBS})
	  IF (NOT EXISTS ${GFLAGS_LIB})
	    MESSAGE(FATAL_ERROR
		    "Can't find Google Flags. Please specify: "
		    "-DGFLAGS_LIB=...")
	  ENDIF (NOT EXISTS ${GFLAGS_LIB})
	  MESSAGE("-- Found Google Flags library: ${GFLAGS_LIB}")
	  FIND_PATH(GFLAGS_INCLUDE NAMES gflags/gflags.h PATHS ${SEARCH_HEADERS})
	  IF (NOT EXISTS ${GFLAGS_INCLUDE})
	    MESSAGE(FATAL_ERROR
		    "Can't find Google Flags. Please specify: "
		    "-DGFLAGS_INCLUDE=...")
	  ENDIF (NOT EXISTS ${GFLAGS_INCLUDE})
	  MESSAGE("-- Found Google Flags header in: ${GFLAGS_INCLUDE}")
	ELSE (GFLAGS)
	  MESSAGE("-- Google Flags disabled; no tests or tools will be built!")
	  ADD_DEFINITIONS(-DCERES_NO_GFLAGS)
	ENDIF (GFLAGS)

	# Google Logging
	MESSAGE("-- Check for Google Log")
	FIND_LIBRARY(GLOG_LIB NAMES glog PATHS ${SEARCH_LIBS})
	IF (NOT EXISTS ${GLOG_LIB})
	  MESSAGE(FATAL_ERROR
		  "Can't find Google Log. Please specify: "
		  "-DGLOG_LIB=...")
	ENDIF (NOT EXISTS ${GLOG_LIB})
	MESSAGE("-- Found Google Log library: ${GLOG_LIB}")

	FIND_PATH(GLOG_INCLUDE NAMES glog/logging.h PATHS ${SEARCH_HEADERS})
	IF (NOT EXISTS ${GLOG_INCLUDE})
	  MESSAGE(FATAL_ERROR
		  "Can't find Google Log. Please specify: "
		  "-DGLOG_INCLUDE=...")
	ENDIF (NOT EXISTS ${GLOG_INCLUDE})
	MESSAGE("-- Found Google Log header in: ${GLOG_INCLUDE}")

	# SuiteSparse/Cholmod
	FIND_PACKAGE(SuiteSparse REQUIRED)
	INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR}) 

	# Ceres
	# Set up the top-level include directories
	SET( CERES_INCLUDE CERES_INCLUDE_DIRECTORY CACHE PATH "Include directory of Ceres")
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${CERES_INCLUDE})
	# Set up the library directory
	SET( CERES_LIB CERES_LIB_DIRECTORY CACHE PATH "Library directory of Ceres")
	LINK_DIRECTORIES( ${LINK_DIRECTORIES} "${CERES_LIB}" )

# Declare the target (an executable)
ADD_EXECUTABLE(PhotoconsistencyFrameAlignment PhotoconsistencyFrameAlignment.cpp
include/CPhotoconsistencyOdometry.h
include/CPhotoconsistencyOdometryCeres.h
third_party/sample.h
include/CPhotoconsistencyOdometryAnalytic.h
)

TARGET_LINK_LIBRARIES(PhotoconsistencyFrameAlignment 
				   ceres
				   ${OpenCV_LIBS}
				   ${GFLAGS_LIB} 
				   ${GLOG_LIB}
				   ${CHOLMOD_LIBRARIES}
				   ${CSPARSE_LIBRARY}
				   )

# Declare the target (an executable)
ADD_EXECUTABLE(PhotoconsistencyVisualOdometry PhotoconsistencyVisualOdometry.cpp
include/CPhotoconsistencyOdometry.h
include/CPhotoconsistencyOdometryCeres.h
third_party/sample.h
include/CPhotoconsistencyOdometryAnalytic.h
./include/CFrameRGBD.h
third_party/cvmat_serialization.h
./include/PointCloudDownsampler.h
./src/PointCloudDownsampler.cpp
./include/CRGBDGrabber.h
./include/CRGBDGrabberRawlog.h
./src/CRGBDGrabberRawlog.cpp
)

TARGET_LINK_LIBRARIES(PhotoconsistencyVisualOdometry 
				   ceres
				   ${OpenCV_LIBS}
				   ${GFLAGS_LIB} 
				   ${GLOG_LIB}
				   ${CHOLMOD_LIBRARIES}
				   ${CSPARSE_LIBRARY}
				   ${PCL_LIBRARIES} 
				   ${MRPT_LIBS}
				   boost_serialization
				   )

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


