cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


# eigen3
INCLUDE_DIRECTORIES ("/usr/local/include/eigen3/" "./test")
find_package(Eigen3 REQUIRED)


# Ceres need CSparse
# SET(CSparse_include_DIR "/usr/local/Cellar/suite-sparse/5.3.0_1/include/")
# SET(CSparse_LIB "/usr/local/Cellar/suite-sparse/5.3.0_1/lib/")
# include_directories( ${CSparse_include_DIR} )
# link_directories( ${CSparse_LIB} )

# SET( Ceres_DIR  "/usr/local/Cellar/ceres-solver/1.14.0_5/lib/cmake/Ceres")
find_package( Ceres REQUIRED )
MESSAGE( STATUS "CERES_INCLUDE_DIRS: " ${CERES_INCLUDE_DIRS} ) # include eigen3 as well
MESSAGE( STATUS "CERES_LIBRARIES: " ${CERES_LIBRARIES} )
include_directories( ${CERES_INCLUDE_DIRS} )


# /usr/local/Cellar/glog/0.4.0
# SET(glog_include_DIR "/usr/local/Cellar/glog/0.4.0/include/")
# SET(glog_LIB "/usr/local/Cellar/glog/0.4.0/lib/")
# find_package( glog )
# include_directories( ${glog_include_DIR})
# link_directories( ${glog_LIB} )

# "/usr/local/Cellar/gflags/2.2.2/include"
# SET(gflags_include_DIR "/usr/local/Cellar/gflags/2.2.2/include/")
# SET(gflags_LIB "/usr/local/Cellar/gflags/2.2.2/lib/")
# find_package( gflags )
# include_directories( ${gflags_include_DIR})
# link_directories( ${gflags_LIB} )


# Pangolin
# SET(Pangolin_DIR "/Users/lingqiujin/Third_Party_Packages/Pangolin/build")
set(Pangolin_DIR "/home/hzhang8/work/github/Pangolin/build")
find_package(Pangolin REQUIRED)

# OpenCV
# SET(OpenCV_DIR "/Users/lingqiujin/Third_Party_Packages/opencv-2.4.13.6/build")
# find_package( OpenCV 2.4 REQUIRED )
# SET(OpenCV_DIR "/usr/local/Cellar/opencv@3/3.4.5_2/share/OpenCV")
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIB_DIR})
MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( STATUS "OpenCV_LIB_DIR: " ${OpenCV_LIB_DIR} )
MESSAGE( STATUS "OpenCV_LIBS: " ${OpenCV_LIBS} )

# find_package( OpenCV REQUIRED )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)


# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(camera_model SHARED
    src/camera_models/chessboard/Chessboard.cc
    src/camera_models/calib/CameraCalibration.cc
    src/camera_models/camera_models/Camera.cc
    src/camera_models/camera_models/CameraFactory.cc
    src/camera_models/camera_models/CostFunctionFactory.cc
    src/camera_models/camera_models/PinholeCamera.cc
    src/camera_models/camera_models/CataCamera.cc
    src/camera_models/camera_models/EquidistantCamera.cc
    src/camera_models/camera_models/ScaramuzzaCamera.cc
    src/camera_models/sparse_graph/Transform.cc
    src/camera_models/gpl/gpl.cc
    src/camera_models/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(camera_model ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


ADD_LIBRARY(MyVio SHARED
    src/System.cpp
    src/parameters.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/feature_tracker.cpp

    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp

    src/backend/vertex.cc
    src/backend/edge.cc
    src/backend/problem.cc
    src/backend/vertex_pose.cc
    src/backend/edge_reprojection.cc
    src/backend/edge_imu.cc
    src/backend/edge_prior.cpp
    src/backend/loss_function.cc
    src/backend/imu_integration.cc
    )

target_link_libraries(MyVio  
  ${OpenCV_LIBS}  
  ${CERES_LIBRARIES}
	${Pangolin_LIBRARIES}
  camera_model)

add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc 
  MyVio  
  -lpthread) 

add_executable(run_sim test/run_sim.cpp test/get_imu.cpp)
target_link_libraries(run_sim MyVio -lpthread)

# add_executable(testCurveFitting test/CurveFitting.cpp)
# target_link_libraries(testCurveFitting MyVio)

