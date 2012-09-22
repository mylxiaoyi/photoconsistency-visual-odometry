PROJECT(phovo)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# Set the PHOVO_SOURCE_DIR variable to the current source directory
set(PHOVO_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set the cmake find libraries directory
set(CMAKE_MODULE_PATH ${PHOVO_SOURCE_DIR}/cmake) 

# Set the configuration file
CONFIGURE_FILE( ${PHOVO_SOURCE_DIR}/config.h.in ${PHOVO_SOURCE_DIR}/phovo/include/config.h )

	# OpenCV library
	FIND_PACKAGE(OpenCV REQUIRED )

	# OpenMP
	FIND_PACKAGE(OpenMP)
	IF(OPENMP_FOUND)
	    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
	ELSE(OPENMP_FOUND)
	    MESSAGE("-- Can't find OpenMP. Continuing without it.")
	ENDIF(OPENMP_FOUND)

	# Ceres and its dependencies
	SET( PHOVO_WITH_CERES OFF CACHE BOOL "Use Ceres to enable CPhotoconsistencyOdometryCeres")
	IF (PHOVO_WITH_CERES)
		# Google Flags
		OPTION(GFLAGS
		       "Enable Google Flags."
		       ON)

		IF (GFLAGS)
		  FIND_LIBRARY(GFLAGS_LIB NAMES gflags PATHS ${SEARCH_LIBS})
		  IF (NOT EXISTS ${GFLAGS_LIB})
		    MESSAGE(FATAL_ERROR
			    "Can't find Google Flags. Please specify: "
			    "-DGFLAGS_LIB=...")
		  ENDIF (NOT EXISTS ${GFLAGS_LIB})
		  FIND_PATH(GFLAGS_INCLUDE NAMES gflags/gflags.h PATHS ${SEARCH_HEADERS})
		  IF (NOT EXISTS ${GFLAGS_INCLUDE})
		    MESSAGE(FATAL_ERROR
			    "Can't find Google Flags. Please specify: "
			    "-DGFLAGS_INCLUDE=...")
		  ENDIF (NOT EXISTS ${GFLAGS_INCLUDE})
		ELSE (GFLAGS)
		  ADD_DEFINITIONS(-DCERES_NO_GFLAGS)
		ENDIF (GFLAGS)

		# Google Logging
		FIND_LIBRARY(GLOG_LIB NAMES glog PATHS ${SEARCH_LIBS})
		IF (NOT EXISTS ${GLOG_LIB})
		  MESSAGE(FATAL_ERROR
			  "Can't find Google Log. Please specify: "
			  "-DGLOG_LIB=...")
		ENDIF (NOT EXISTS ${GLOG_LIB})

		FIND_PATH(GLOG_INCLUDE NAMES glog/logging.h PATHS ${SEARCH_HEADERS})
		IF (NOT EXISTS ${GLOG_INCLUDE})
		  MESSAGE(FATAL_ERROR
			  "Can't find Google Log. Please specify: "
			  "-DGLOG_INCLUDE=...")
		ENDIF (NOT EXISTS ${GLOG_INCLUDE})

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
	ENDIF(PHOVO_WITH_CERES)

# Add the "phovo" library subdirectory
ADD_SUBDIRECTORY(phovo)

# Select the "apps" to be built
SET( BUILD_PHOTOCONSISTENCY_FRAME_ALIGNMENT_APPLICATION ON CACHE BOOL "Build PhotoconsistencyFrameAlignment application")
SET( BUILD_PHOTOCONSISTENCY_VISUAL_ODOMETRY_APPLICATION OFF CACHE BOOL "Build PhotoconsistencyVisualOdometry application")

# Add the "apps" subdirectory
ADD_SUBDIRECTORY(apps)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


