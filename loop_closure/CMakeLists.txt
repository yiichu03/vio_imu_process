cmake_minimum_required(VERSION 3.0.0)
project(loop_closure_module VERSION 1.0.0 LANGUAGES CXX)

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
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../okvis/cmake")

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
  tf)

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

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS})

# require OpenCV
find_package(OpenCV COMPONENTS core highgui imgproc features2d REQUIRED)
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

# require okvis
find_package(okvis 1.1 REQUIRED)
include_directories(${OKVIS_INCLUDE_DIRS})

# require brisk
find_package(brisk 2 REQUIRED)
include_directories(${BRISK_INCLUDE_DIRS})

# require OpenGV
find_package(opengv REQUIRED)

find_package(Ceres REQUIRED)

find_package(Sophus)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(DBoW2)
include_directories(${DBoW2_INCLUDE_DIRS})

find_package(GTSAM)
if(GTSAM_FOUND)
  add_definitions(-DHAVE_GTSAM)
  include_directories(${GTSAM_INCLUDE_DIR})
  find_package(KimeraRPGO REQUIRED)

  list(APPEND LINK_LIBRARIES
    gtsam
    gtsam_unstable KimeraRPGO)
  message(STATUS "Found GTSAM at ${GTSAM_INCLUDE_DIR}.")
else()
  message(WARNING "GTSAM not found")
endif()

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

### Download and unzip the vocabularly file
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.yml)
  message(STATUS "Downloading vocabulary file from dropbox.")
  file(DOWNLOAD
       http://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=1
       ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
       SHOW_PROGRESS
       STATUS orbvoc_download_success
       TIMEOUT 60
       EXPECTED_MD5 41c290ca05ab2b68548dc6542632d69f)
  if(orbvoc_download_success)
    message(STATUS "Unzipping vocabulary file.")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.zip
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    message(STATUS "Moving vocabulary file.")
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/
                    ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/)
    execute_process(COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc/)
  else(orbvoc_download_success)
    message(STATUS "Failed to download vocabulary file. Please download manually.")
  endif(orbvoc_download_success)
else()
  message(STATUS "Vocabulary file exists, will not download.")
endif()

# compile the ROS dependent stuff into a library
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

list(APPEND SOURCEFILES
    src/LcdThirdPartyWrapper.cpp
    src/LoopClosureDetectorParams.cpp
    src/FactoryMethods.cpp
    src/PgoPublisher.cpp
    src/LoopClosureModule.cpp
    src/LoopClosureMethod.cpp
    src/LoopClosureParameters.cpp
)

if(GTSAM_FOUND)
list(APPEND SOURCEFILES
    src/GtsamWrap.cpp
    src/LoopClosureDetector.cpp
)
endif(GTSAM_FOUND)

add_library(${PROJECT_NAME} STATIC
  ${SOURCEFILES}
)

list(APPEND LINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${OKVIS_LIBRARIES}
    ${DBoW2_LIBRARIES}
    ${GLOG_LIBRARY}
    pthread
    gflags
)

target_link_libraries(${PROJECT_NAME}  
  ${LINK_LIBRARIES}
)

# now the actual applications
add_executable(loop_closure_node src/loop_closure_node.cpp)
target_link_libraries(loop_closure_node ${PROJECT_NAME})

set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

list(APPEND TEST_SRC_FILES
    test/test_main.cpp
    test/TestLcdThirdPartyWrapper.cpp
)

if(GTSAM_FOUND)
list(APPEND TEST_SRC_FILES
    test/TestGtsamBetweenFactor.cpp
    test/TestLoopClosureMethod.cpp
    test/TestRPGO.cpp
    test/TestSimpleProjectionFactor.cpp
)
endif(GTSAM_FOUND)

# In a catkin package, do not use or link gtest which is supported poorly.
if(CATKIN_ENABLE_TESTING)
  set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
  catkin_add_gtest(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})
  target_link_libraries(${PROJECT_TEST_NAME} ${PROJECT_NAME}  ${Boost_LIBRARIES})
endif()
