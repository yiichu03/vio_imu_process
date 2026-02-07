cmake_minimum_required(VERSION 3.0.0)
project(sliding_window_estimator VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
  add_definitions(-DNDEBUG)
endif()

message(STATUS "Building ${PROJECT_NAME} with ${CMAKE_BUILD_TYPE}")

option(DO_TIMING "Time routines." OFF)
if(NOT DO_TIMING)
  add_definitions(-DDEACTIVATE_TIMERS)
  message(STATUS "Deactivating timers.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# https://github.com/StableCoder/cmake-scripts.git
include(../cmake/sanitizers.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
else()
  message(STATUS "Assuming SSE instructions available.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
endif()

list(APPEND ROS_DEPENDENT_PACKAGES
  rosbag
  roscpp
  sensor_msgs
  csfm_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  tf
  vio_common)

list(APPEND BSPLINE_PACKAGES
  bsplines
  sm_common
  sm_eigen
  sm_kinematics
  sm_timing
  sparse_block_matrix)

if(($ENV{ROS_DISTRO} STREQUAL "melodic") OR ($ENV{ROS_DISTRO} STREQUAL "noetic"))
  LIST(APPEND ROS_DEPENDENT_PACKAGES
    pcl_conversions
    pcl_ros)
  add_definitions(-DHAVE_PCL)
elseif($ENV{ROS_DISTRO} STREQUAL "kinetic")
  message("Publishing with PCL point cloud is disabled with ROS kinetic because"
      " libpcl-dev on Ubuntu 16 depends on the system-wide Eigen \nwhich"
      " is incompatible with ceres solver 14.0.")
else()
  message("Unknown ROS distro: $ENV{ROS_DISTRO}.")
endif()

find_package(catkin
  COMPONENTS ${ROS_DEPENDENT_PACKAGES}
  OPTIONAL_COMPONENTS ${BSPLINE_PACKAGES}
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

if(bsplines_FOUND)
  add_definitions(-DHAVE_BSPLINES)
else()
  message(WARNING "bsplines not found. Simulation with B Spline fitting will be disabled!")
endif()

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# require OpenCV
# find_package(OpenCV COMPONENTS core highgui imgproc features2d REQUIRED)
# include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# require okvis
find_package(okvis 1.1 REQUIRED)
include_directories(${OKVIS_INCLUDE_DIRS})

# require brisk
find_package(brisk 2 REQUIRED)
include_directories(${BRISK_INCLUDE_DIRS})

# require OpenGV
find_package(opengv REQUIRED)

# Find OpenMP
if(APPLE)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_LIB_NAMES "/usr/local/opt/libomp/lib/libomp.dylib")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
      set(OpenMP_CXX_LIB_NAMES "/usr/local/opt/libomp/lib/libomp.dylib")
      set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
    set(OpenMP_FOUND TRUE)
else()
    find_package(OpenMP REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS "${CMAKE_CXX_FLAGS_EXTERNAL_PROJECTS} ${OpenMP_CXX_FLAGS}")
endif()

FIND_PACKAGE(SUITESPARSE)
include_directories(${SUITESPARSE_INCLUDE_DIRS})

find_package(Ceres REQUIRED)

find_package(Sophus)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(GTSAM CONFIG REQUIRED)
find_package(GTSAM_UNSTABLE CONFIG REQUIRED)

# If exported targets exist, just use them (best)
if(TARGET gtsam AND TARGET gtsam_unstable)
  add_definitions(-DHAVE_GTSAM)
  list(APPEND LINK_LIBRARIES gtsam gtsam_unstable)
else()
  message(FATAL_ERROR "GTSAM targets not exported (expected targets: gtsam, gtsam_unstable). Check GTSAM-exports.cmake.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# https://github.com/Crascit/DownloadProject/blob/master/CMakeLists.txt
if(CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

# compile the ROS dependent stuff into a library
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

list(APPEND SOURCEFILES
    src/feature_tracker/distance.cpp
    src/feature_tracker/FeatureTracker.cpp
    src/feature_tracker/Feature2d2dMatchingAlgorithm.cpp
    src/feature_tracker/SimpleFrameRelativeAdapter.cpp
    src/feature_tracker/StereoMatchingAlgorithm.cpp

    src/gtsam/VioBackEndParams.cpp
    src/gtsam/ImuFactorTestHelpers.cpp

    src/io_wrap/CameraPoseVisualization.cpp
    src/io_wrap/CommonGflags.cpp
    src/io_wrap/Player.cpp
    src/io_wrap/PoseViewer.cpp
    src/io_wrap/Publisher.cpp
    src/io_wrap/RosParametersReader.cpp
    src/io_wrap/StreamHelper.cpp
    src/io_wrap/Subscriber.cpp

    src/swift_vio/FilterBase.cpp
    src/swift_vio/EkfUpdater.cpp
    src/swift_vio/EstimatorBase.cpp
    src/swift_vio/EstimationHelper.cpp
    src/swift_vio/ExternalFramewiseFrontend.cpp
    src/swift_vio/FrameSynchronizer.cpp
    src/swift_vio/FeatureTrackingFrontend.cpp
    src/swift_vio/FeatureTriangulation.cpp
    src/swift_vio/FilterHelper.cpp
    src/swift_vio/filterJacobians.cpp
    src/swift_vio/FramewiseDescriptorFrontend.cpp
    src/swift_vio/ImuInitializer.cpp
    src/swift_vio/KeyframePublisher.cpp
    src/swift_vio/KltFrontend.cpp
    src/swift_vio/messages.cpp
    src/swift_vio/MotionAndStructureStats.cpp
    src/swift_vio/PointLandmark.cpp
    src/swift_vio/SlidingWindowFilter.cpp
    src/swift_vio/SlidingWindowSmoother.cpp
    src/swift_vio/StatAccumulator.cpp
    src/swift_vio/TimingUtilities.cpp
    src/swift_vio/triangulate.cpp
    src/swift_vio/TwoViewPair.cpp
    src/swift_vio/ViBundleAdjustment.cpp
    src/swift_vio/VioEvaluationCallback.cpp
    src/swift_vio/VioFactoryMethods.cpp
    src/swift_vio/VioSystem.cpp
    src/swift_vio/VioSystemBase.cpp
    src/swift_vio/VioVisualizer.cpp
    src/swift_vio/VisualMap.cpp

    include/simul/SimParameters.h
    src/simul/CameraSystemCreator.cpp
    src/simul/CoupledSimulationFrontend.cpp
    src/simul/curves.cpp
    src/simul/ImuNoiseSimulator.cpp
    src/simul/LandmarkGrid.cpp
    src/simul/numeric_ceres_residual_Jacobian.cpp
    src/simul/PointLandmarkSimulationRS.cpp
    src/simul/simGflags.cpp
    src/simul/SimulatorBase.cpp
    src/simul/SimulationFrontend.cpp
    src/simul/SimParametersReader.cpp
    src/simul/VioSimTestSystem.cpp
)

if(GTSAM_FOUND)
list(APPEND SOURCEFILES
    src/gtsam/ImuFrontEnd.cpp
    src/gtsam/ImuFrontEndParams.cpp
    # src/gtsam/RiProjectionFactorIDP.cpp
    # src/gtsam/RiProjectionFactorIDPAnchor.cpp
    # src/gtsam/RiExtendedPose3.cpp
    # src/gtsam/RiExtendedPose3Prior.cpp
    # src/gtsam/RiImuFactor.cpp
    # src/gtsam/RiFixedLagSmoother.cpp
    # src/gtsam/FixedLagSmoother.cpp
)
endif(GTSAM_FOUND)

add_library(${PROJECT_NAME} STATIC
  ${SOURCEFILES}
)

list(APPEND LINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${OKVIS_LIBRARIES}
    ${BRISK_LIBRARIES}
    ${CERES_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${OpenMP_LIBRARY}
    pthread
)

target_link_libraries(${PROJECT_NAME}  
  ${LINK_LIBRARIES}
)

# now the actual applications
add_executable(swift_vio_node src/apps/swift_vio_node.cpp)
target_link_libraries(swift_vio_node ${PROJECT_NAME})

add_executable(swift_vio_node_synchronous src/apps/swift_vio_node_synchronous.cpp)
target_link_libraries(swift_vio_node_synchronous ${PROJECT_NAME})

add_executable(okvis_node_synchronous src/apps/okvis_node_synchronous.cpp)
target_link_libraries(okvis_node_synchronous ${PROJECT_NAME})

add_executable(swift_vio_simulation src/simul/swift_vio_simulation.cpp)
target_link_libraries(swift_vio_simulation ${PROJECT_NAME} gtest gflags)

add_executable(swift_vio_sim_mono src/simul/swift_vio_sim_mono.cpp)
target_link_libraries(swift_vio_sim_mono ${PROJECT_NAME} gtest gflags)

add_executable(feature_tracking_frontend_test src/simul/TestFeatureTrackingFrontend.cpp)
target_link_libraries(feature_tracking_frontend_test ${PROJECT_NAME} gtest gflags)

add_executable(time_rotation_calibrator src/apps/time_rotation_calibrator.cpp
    src/swift_vio/TimeAndRotationCalibrator.cpp)
target_link_libraries(time_rotation_calibrator okvis_util okvis_time okvis_timing
    okvis_common okvis_kinematics okvis_ceres ${vio_common_LIBRARIES})

add_executable(vi_bundle_adjustment src/apps/vi_bundle_adjustment.cpp)
target_link_libraries(vi_bundle_adjustment ${PROJECT_NAME})

add_executable(propag_by_preint src/apps/propag_by_preint.cpp)
target_link_libraries(propag_by_preint ${PROJECT_NAME} GTest::gtest)

add_executable(gtsam_ref_preint_from_txt src/apps/gtsam_ref_preint_from_txt.cpp)
target_link_libraries(gtsam_ref_preint_from_txt ${PROJECT_NAME})

add_executable(compare_preint_outputs src/apps/compare_preint_outputs.cpp)
target_link_libraries(compare_preint_outputs ${PROJECT_NAME})

add_executable(compare_vinsmono_gtsam src/apps/compare_vinsmono_gtsam.cpp)
target_link_libraries(compare_vinsmono_gtsam ${PROJECT_NAME})

add_executable(imu_file_smoketest src/apps/imu_file_smoketest.cpp)
target_link_libraries(imu_file_smoketest ${PROJECT_NAME} gflags)

find_package(Boost COMPONENTS filesystem system thread REQUIRED)
add_executable(average_transformations src/apps/average_transformations.cpp)
target_link_libraries(average_transformations  ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

list(APPEND TEST_SRC_FILES
    test/test_main.cpp

    test/boost/TestBoostAccumulators.cpp
    test/boost/TestChiSquared.cpp

    test/eigen/TestEigen2CV.cpp
    test/eigen/TestEigenMatrixInitialization.cpp
    test/eigen/TestEigenQR.cpp
    test/eigen/TestEigenQuaternion.cpp

    test/io_wrap/TestVioDatasetPlayer.cpp

    test/swift_vio/SmootherJacobianUtilities.cpp
    test/swift_vio/TestBoundedImuDeque.cpp
    test/swift_vio/TestCovarianceEstimation.cpp
    test/swift_vio/TestEkf.cpp
    test/swift_vio/TestEuclideanParamErrorSized.cpp
    test/swift_vio/TestExtrinsicJacobian.cpp
    test/swift_vio/TestFeatureInitialization.cpp
    test/swift_vio/TestFilterHelper.cpp
    test/swift_vio/TestSwiftVio.cpp
    test/swift_vio/TestImuOdometry.cpp
    test/swift_vio/TestCurves.cpp
    test/swift_vio/TestMergeTwoLandmarks.cpp
    test/swift_vio/TestOkvisTriangulateFast.cpp
    test/swift_vio/TestOkvisTime.cpp
    test/swift_vio/TestPointLandmark.cpp
    test/swift_vio/TestPointLandmarkSimulationRS.cpp
    test/swift_vio/TestQuaternionFromSmallAngle.cpp
    test/swift_vio/TestRelativeMotionJacobian.cpp
    test/swift_vio/TestRsReprojectionError.cpp
    test/swift_vio/TestSE3Interpolation.cpp
    src/swift_vio/TimeAndRotationCalibrator.cpp
    test/swift_vio/TestTimeAndRotationCalibrator.cpp
    test/swift_vio/TestTriangulate.cpp
    test/swift_vio/TestTwoViewPair.cpp

    test/opencv/TestOpenCVDistortion.cpp

    test/ros/TestRosbag.cpp

    test/std/TestBinaryOperators.cpp
    test/std/TestCallMemberFunctionAtConstruction.cpp
    test/std/TestConstStaticInBaseClass.cpp
    test/std/TestDequeIterator.cpp
    test/std/TestDerivedClass.cpp
    test/std/TestMapKeyOrder.cpp
    test/std/TestModifyThisInConstMethod.cpp
    test/std/TestPointerCompare.cpp
    test/std/TestSizeof.cpp
    test/std/TestStdAccumulate.cpp
)

if(GTSAM_FOUND)
list(APPEND TEST_SRC_FILES
    test/gtsam/TestTriangulate.cpp
    test/gtsam/TestGtsamImuFactor.cpp
    test/gtsam/TestImuFrontEnd.cpp
    test/gtsam/TestIncrementalFixedLagSmoother.cpp
    test/gtsam/TestRiExtendedPose3.cpp
    test/gtsam/TestRiExtendedPose3Prior.cpp
    test/gtsam/TestRiImuFactor.cpp
    test/gtsam/TestRiProjectionFactorIDP.cpp
    test/gtsam/TestRiSmartProjectionFactor.cpp
    test/gtsam/TestRiTransformPointJacobian.cpp
)
endif(GTSAM_FOUND)

# In a catkin package, do not use or link gtest which is supported poorly.
if(CATKIN_ENABLE_TESTING)
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  catkin_add_gtest(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
  target_link_libraries(${PROJECT_TEST_NAME} ${PROJECT_NAME}  ${Boost_LIBRARIES})
endif()
