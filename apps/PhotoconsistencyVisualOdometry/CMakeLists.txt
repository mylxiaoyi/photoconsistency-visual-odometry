# PCL library
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# MRPT library
FIND_PACKAGE(MRPT REQUIRED hwdrivers maps graphslam) 

# Include the phovo include files
INCLUDE_DIRECTORIES(${PHOVO_SOURCE_DIR}/phovo/include)

# Include the third party include files
INCLUDE_DIRECTORIES(${PHOVO_SOURCE_DIR}/third_party)

# Make sure the linker can find the "phovo" library once it is built.
link_directories (${PHOVO_BINARY_DIR}/phovo) 

# Declare the target (an executable)
ADD_EXECUTABLE(PhotoconsistencyVisualOdometry PhotoconsistencyVisualOdometry.cpp
	${PHOVO_SOURCE_DIR}/third_party/cvmat_serialization.h	
	include/CFrameRGBD.h
	include/PointCloudDownsampler.h
	src/PointCloudDownsampler.cpp
	include/CRGBDGrabber.h
	include/CRGBDGrabberRawlog.h
	src/CRGBDGrabberRawlog.cpp
	include/CRGBDGrabberOpenNI_PCL.h
	src/CRGBDGrabberOpenNI_PCL.cpp
)

IF(PHOVO_WITH_CERES)
        TARGET_LINK_LIBRARIES(PhotoconsistencyVisualOdometry phovo 
							     ceres 
							     ${OpenCV_LIBS}
							     ${PCL_LIBRARIES} 
				                             ${MRPT_LIBS}
				                             boost_serialization
							     ${GFLAGS_LIB} 
		                                             ${GLOG_LIB}
		                                             ${CHOLMOD_LIBRARIES}
		                                             ${CSPARSE_LIBRARY})
ELSE(PHOVO_WITH_CERES)
	TARGET_LINK_LIBRARIES(PhotoconsistencyVisualOdometry phovo 
							     ${OpenCV_LIBS}
                                                             ${PCL_LIBRARIES} 
				                             ${MRPT_LIBS}
				                             boost_serialization)
ENDIF(PHOVO_WITH_CERES)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")








