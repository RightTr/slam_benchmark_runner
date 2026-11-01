cmake_minimum_required(VERSION 3.10)
project(orb_slam3)

include(CTest)

get_filename_component(ORB_SLAM3_WRAPPER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(ORB_SLAM3_ROOT_DEFAULT "${ORB_SLAM3_WRAPPER_ROOT}/.." REALPATH)
set(ORB_SLAM3_ROOT "${ORB_SLAM3_ROOT_DEFAULT}" CACHE PATH "Path to the native ORB-SLAM3 source tree")
get_filename_component(ORB_SLAM3_ROOT "${ORB_SLAM3_ROOT}" REALPATH)
set(ORB_SLAM3_BUILD_ROOT "${CMAKE_BINARY_DIR}/orbslam3_core" CACHE PATH "Workspace build directory for native ORB-SLAM3 libraries")
if(NOT EXISTS "${ORB_SLAM3_ROOT}/include/System.h")
  message(FATAL_ERROR "ORB_SLAM3_ROOT does not contain include/System.h: ${ORB_SLAM3_ROOT}")
endif()



find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 PATHS "${ORB_SLAM3_BUILD_ROOT}/lib" NO_DEFAULT_PATH)
find_library(DBOW2_LIBRARY NAMES DBoW2 PATHS
  "${ORB_SLAM3_BUILD_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(G2O_LIBRARY NAMES g2o PATHS "${ORB_SLAM3_BUILD_ROOT}/Thirdparty/g2o/lib" NO_DEFAULT_PATH)
if(NOT ORB_SLAM3_LIBRARY OR NOT DBOW2_LIBRARY OR NOT G2O_LIBRARY)
  message(FATAL_ERROR "Build the native ORB-SLAM3 library and Thirdparty libraries first (run ../build.sh)")
endif()

set(ORB_SLAM3_INCLUDE_DIRS
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2"
  "${ORB_SLAM3_ROOT}/Thirdparty/g2o"
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)
set(ORB_SLAM3_LINK_LIBRARIES
  "${ORB_SLAM3_LIBRARY}" "${DBOW2_LIBRARY}" "${G2O_LIBRARY}"
  ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} -lboost_serialization -lcrypto
)
set(WRAPPER_RPATH "${ORB_SLAM3_BUILD_ROOT}/lib;${ORB_SLAM3_BUILD_ROOT}/Thirdparty/g2o/lib")
if(TARGET pango_display)
  get_target_property(PANGOLIN_DISPLAY_LIBRARY pango_display IMPORTED_LOCATION_RELEASE)
  if(NOT PANGOLIN_DISPLAY_LIBRARY)
    get_target_property(PANGOLIN_DISPLAY_LIBRARY pango_display IMPORTED_LOCATION)
  endif()
  if(PANGOLIN_DISPLAY_LIBRARY)
    get_filename_component(PANGOLIN_LIBRARY_DIR "${PANGOLIN_DISPLAY_LIBRARY}" DIRECTORY)
    list(APPEND WRAPPER_RPATH "${PANGOLIN_LIBRARY_DIR}")
  endif()
endif()

# Pangolin's shared libraries depend on one another. Keep their directory
# available to the loader while resolving those indirect dependencies too.
if(UNIX AND NOT APPLE)
  set(PANGOLIN_TRANSITIVE_RPATH_FLAGS "-Wl,--disable-new-dtags")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(slam_bench_msgs REQUIRED)

  foreach(node stereo rgbd stereo_inertial rgbd_inertial)
    add_executable(ros_${node} "${CMAKE_CURRENT_SOURCE_DIR}/src/${node}.cpp")
    if(node STREQUAL "rgbd")
      target_sources(ros_${node} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/benchmark_bridge.cpp")
      ament_target_dependencies(ros_${node} slam_bench_msgs)
    endif()
    target_compile_definitions(ros_${node} PRIVATE USE_ROS2)
    target_include_directories(ros_${node} PRIVATE ${ORB_SLAM3_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")
    ament_target_dependencies(ros_${node} rclcpp sensor_msgs builtin_interfaces cv_bridge message_filters nav_msgs geometry_msgs std_msgs tf2_ros visualization_msgs)
    target_link_libraries(ros_${node} ${ORB_SLAM3_LINK_LIBRARIES})
    set_target_properties(ros_${node} PROPERTIES INSTALL_RPATH "${WRAPPER_RPATH}" BUILD_RPATH "${WRAPPER_RPATH}" LINK_FLAGS "${PANGOLIN_TRANSITIVE_RPATH_FLAGS}")
    install(TARGETS ros_${node} DESTINATION lib/${PROJECT_NAME})
  endforeach()
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
