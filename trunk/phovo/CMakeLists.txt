# Create the PhotoconsistencyVisualOdometry library
ADD_LIBRARY(phovo
		../third_party/sample.h
		include/CPhotoconsistencyOdometry.h
		include/CPhotoconsistencyOdometryCeres.h
		include/CPhotoconsistencyOdometryAnalytic.h)

# Link the libraries used by PhotoconsistencyVisualOdometry
LINK_LIBRARIES(phovo 
		ceres
		${OpenCV_LIBS}
		${GFLAGS_LIB} 
		${GLOG_LIB}
		${CHOLMOD_LIBRARIES}
		${CSPARSE_LIBRARY})

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(phovo PROPERTIES LINKER_LANGUAGE CXX)

