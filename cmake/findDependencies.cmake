
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

message(STATUS "OpenCV found, version: ${OpenCV_VERSION} in dir ${OpenCV_INCLUDE_DIRS}")

if(NOT USE_OWN_EIGEN3)
    message(WARNING "If you do not want to install Eigen you can turn on the option USE_OWN_EIGEN3")
    find_package( Eigen3 REQUIRED )
else()
    set(EIGEN3_INCLUDE_DIR "3rdparty/eigen3")
endif()
include_directories( ${EIGEN3_INCLUDE_DIR} )

if(USE_DOUBLE_PRECISION_PNP)
    add_definitions(-DDOUBLE_PRECISION_PNP)
endif()
