# Install script for directory: /home/user/cartographer_ws_docker/src/cartographer_ros/cartographer_rviz

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/user/cartographer_ws_docker/install/cartographer_rviz")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/user/cartographer_ws_docker/build/cartographer_rviz/ament_cmake_symlink_install/ament_cmake_symlink_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/user/cartographer_ws_docker/build/cartographer_rviz/libcartographer_rviz.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so"
         OLD_RPATH "/home/user/cartographer_ws_docker/install/cartographer_ros_msgs/lib:/home/user/tracing_ws/install/nav_msgs/lib:/home/user/tracing_ws/install/rosgraph_msgs/lib:/home/user/tracing_ws/install/rcl_yaml_param_parser/lib:/home/user/tracing_ws/install/statistics_msgs/lib:/home/user/tracing_ws/install/tracetools/lib:/home/user/tracing_ws/install/message_filters/lib:/home/user/tracing_ws/install/rclcpp/lib:/home/user/tracing_ws/install/tf2/lib:/home/user/tracing_ws/install/action_msgs/lib:/home/user/tracing_ws/install/tf2_msgs/lib:/home/user/tracing_ws/install/tf2_ros/lib:/home/user/tracing_ws/install/builtin_interfaces/lib:/home/user/tracing_ws/install/geometry_msgs/lib:/home/user/tracing_ws/install/sensor_msgs/lib:/home/user/tracing_ws/install/std_msgs/lib:/home/user/tracing_ws/install/rosidl_typesupport_fastrtps_c/lib:/home/user/tracing_ws/install/rmw/lib:/home/user/tracing_ws/install/rosidl_typesupport_fastrtps_cpp/lib:/home/user/tracing_ws/install/rcpputils/lib:/home/user/tracing_ws/install/rosidl_typesupport_c/lib:/home/user/tracing_ws/install/rosidl_typesupport_introspection_c/lib:/home/user/tracing_ws/install/rcutils/lib:/home/user/tracing_ws/install/visualization_msgs/lib:/home/user/tracing_ws/install/rosidl_runtime_c/lib:/home/user/tracing_ws/install/rosidl_typesupport_cpp/lib:/home/user/tracing_ws/install/rosidl_typesupport_introspection_cpp/lib:/home/user/tracing_ws/install/rosbag2_cpp/lib:/home/user/tracing_ws/install/rosbag2_storage/lib:/home/user/tracing_ws/install/urdf/lib:/home/user/tracing_ws/install/rviz_common/lib:/home/user/tracing_ws/install/rviz_rendering/lib:/home/user/tracing_ws/install/resource_retriever/lib:/home/user/tracing_ws/install/rclcpp_action/lib:/home/user/tracing_ws/install/libstatistics_collector/lib:/home/user/tracing_ws/install/rcl_action/lib:/home/user/tracing_ws/install/rcl/lib:/home/user/tracing_ws/install/rcl_interfaces/lib:/home/user/tracing_ws/install/libyaml_vendor/lib:/home/user/tracing_ws/install/rmw_implementation/lib:/home/user/tracing_ws/install/rcl_logging_spdlog/lib:/home/user/tracing_ws/install/rcl_logging_interface/lib:/home/user/tracing_ws/install/unique_identifier_msgs/lib:/home/user/tracing_ws/install/fastcdr/lib:/home/user/tracing_ws/install/class_loader/lib:/home/user/tracing_ws/install/ament_index_cpp/lib:/opt/ros/humble/lib:/home/user/tracing_ws/install/urdfdom/lib:/home/user/tracing_ws/install/rviz_ogre_vendor/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcartographer_rviz.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/cartographer_ws_docker/install/cartographer_rviz/share/cartographer_rviz/ogre_media/materials/glsl120")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/user/cartographer_ws_docker/install/cartographer_rviz/share/cartographer_rviz/ogre_media/materials" TYPE DIRECTORY FILES "/home/user/cartographer_ws_docker/src/cartographer_ros/cartographer_rviz/ogre_media/materials/glsl120" USE_SOURCE_PERMISSIONS)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/user/cartographer_ws_docker/install/cartographer_rviz/share/cartographer_rviz/ogre_media/materials/scripts")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/user/cartographer_ws_docker/install/cartographer_rviz/share/cartographer_rviz/ogre_media/materials" TYPE DIRECTORY FILES "/home/user/cartographer_ws_docker/src/cartographer_ros/cartographer_rviz/ogre_media/materials/scripts" USE_SOURCE_PERMISSIONS)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake/cartographer_rvizExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake/cartographer_rvizExport.cmake"
         "/home/user/cartographer_ws_docker/build/cartographer_rviz/CMakeFiles/Export/share/cartographer_rviz/cmake/cartographer_rvizExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake/cartographer_rvizExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake/cartographer_rvizExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake" TYPE FILE FILES "/home/user/cartographer_ws_docker/build/cartographer_rviz/CMakeFiles/Export/share/cartographer_rviz/cmake/cartographer_rvizExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer_rviz/cmake" TYPE FILE FILES "/home/user/cartographer_ws_docker/build/cartographer_rviz/CMakeFiles/Export/share/cartographer_rviz/cmake/cartographer_rvizExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/user/cartographer_ws_docker/build/cartographer_rviz/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
