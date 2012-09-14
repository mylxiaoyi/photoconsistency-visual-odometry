# Make sure the linker can find the "phovo" library once it is built.
link_directories (${PHOVO_BINARY_DIR}/phovo) 

# Declare the target (an executable)
ADD_EXECUTABLE(PhotoconsistencyFrameAlignment PhotoconsistencyFrameAlignment.cpp)

TARGET_LINK_LIBRARIES(PhotoconsistencyFrameAlignment phovo ${OpenCV_LIBS})

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


