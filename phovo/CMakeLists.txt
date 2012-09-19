# Create the PhotoconsistencyVisualOdometry library
IF(PHOVO_WITH_CERES)
	ADD_LIBRARY(phovo
		    include/CPhotoconsistencyOdometry.h
		    include/CPhotoconsistencyOdometryAnalytic.h
		    include/CPhotoconsistencyOdometryCeres.h
		    ../third_party/sample.h)	
ELSE(PHOVO_WITH_CERES)
	ADD_LIBRARY(phovo
		    include/CPhotoconsistencyOdometry.h
		    include/CPhotoconsistencyOdometryAnalytic.h)
ENDIF(PHOVO_WITH_CERES)

# Link the libraries used by PhotoconsistencyVisualOdometry
IF(PHOVO_WITH_CERES)
	LINK_LIBRARIES(phovo 
		       ceres
		       ${OpenCV_LIBS}
		       ${GFLAGS_LIB} 
		       ${GLOG_LIB}
		       ${CHOLMOD_LIBRARIES}
		       ${CSPARSE_LIBRARY})
ELSE(PHOVO_WITH_CERES)
	LINK_LIBRARIES(phovo ${OpenCV_LIBS})
ENDIF(PHOVO_WITH_CERES)

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(phovo PROPERTIES LINKER_LANGUAGE CXX)

