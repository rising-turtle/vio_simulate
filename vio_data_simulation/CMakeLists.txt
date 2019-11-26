SET (PROJECT_NAME imuSimWithPointLine)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")


INCLUDE_DIRECTORIES ("/usr/local/include/")

find_package(Eigen3 REQUIRED)

# OpenCV
# SET(OpenCV_DIR "/usr/local/Cellar/opencv@3/3.4.5_2/share/OpenCV")
find_package( OpenCV REQUIRED) #3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIB_DIR})

# Sophus
# SET(Sophus_DIR "/Users/jin/Third_Party_Packages/Sophus/build")
find_package( Sophus REQUIRED )
message( STATUS "Sophus_INCLUDE_DIR: " ${Sophus_INCLUDE_DIR})
message( STATUS "Sophus_LIBRARY: " ${Sophus_LIBRARY})
include_directories( ${Sophus_INCLUDE_DIR} )

include_directories(
${EIGEN3_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR} # for both sophus and geographiclib
${OPENCV_INCLUDE_DIR}
)

LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
${Sophus_LIBRARIES}
)

ADD_EXECUTABLE(data_gen main/gener_alldata.cpp src/param.h src/param.cpp src/utilities.h src/utilities.cpp src/imu.h src/imu.cpp)
TARGET_LINK_LIBRARIES (data_gen ${LINK_LIBS})

ADD_EXECUTABLE(vinsData main/vins_Simdata.cpp src/param.h src/param.cpp src/utilities.h src/utilities.cpp src/imu.h src/imu.cpp)
TARGET_LINK_LIBRARIES (vinsData ${LINK_LIBS})

ADD_EXECUTABLE(vinsData_depth main/vins_simdata_with_depth.cpp src/param.h src/param.cpp src/utilities.h src/utilities.cpp src/imu.h src/imu.cpp)
TARGET_LINK_LIBRARIES (vinsData_depth ${LINK_LIBS})
