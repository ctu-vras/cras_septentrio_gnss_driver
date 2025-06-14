# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(cras_septentrio_gnss_driver)

find_package(ament_cmake QUIET)
find_package(catkin QUIET)  # catkin_lint: ignore missing_required

if(catkin_FOUND)
  set(CMAKE_CXX_STANDARD 17)

  find_package(catkin REQUIRED COMPONENTS
    compass_msgs
    cras_cpp_common
    geometry_msgs
    gps_common
    nmea_msgs
    pluginlib
    roscpp
    sensor_msgs
    septentrio_gnss_driver
    tf2
    tf2_geometry_msgs
  )

  catkin_package()

  include_directories(${catkin_INCLUDE_DIRS})

  add_library(cras_septentrio_process nodelets/septentrio_process.cpp)
  add_dependencies(cras_septentrio_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(cras_septentrio_process ${catkin_LIBRARIES})
  cras_node_from_nodelet(cras_septentrio_process cras::SeptentrioProcess OUTPUT_NAME process)

  install(TARGETS cras_septentrio_process
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN *.launch
  )

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    PATTERN *ros2.yaml EXCLUDE
  )

  install(FILES
    80-cras-septentrio-gps.rules nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  if(CATKIN_ENABLE_TESTING)
    find_package(roslint REQUIRED)
    find_package(roslaunch REQUIRED)

    roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

    # catkin_lint - checks validity of package.xml and CMakeLists.txt
    roslint_custom(catkin_lint -W2 .)

    # Roslint C++ - checks formatting and some other rules for C++ files
    file(GLOB_RECURSE ROSLINT_SRC nodelets/*.cpp)

    set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
            -build/header_guard,-readability/namespace,-whitespace/braces,-runtime/references,\
            -build/c++11,-readability/nolint,-readability/todo,-legal/copyright,-build/namespaces")
    roslint_cpp(${ROSLINT_SRC})

    roslint_add_test()
  endif()
elseif(ament_cmake_FOUND)
  #catkin_lint: skip
  set(CMAKE_CXX_STANDARD 20)

  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_SYMLINK_INSTALL)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(compass_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(gps_msgs REQUIRED)
  find_package(nmea_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(septentrio_gnss_driver REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  set(dependencies
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${compass_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    ${gps_msgs_TARGETS}
    ${nmea_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${septentrio_gnss_driver_TARGETS}
  )

  add_library(cras_septentrio_process SHARED nodelets/septentrio_process.cpp)
  target_link_libraries(cras_septentrio_process ${dependencies})
  target_compile_definitions(cras_septentrio_process PRIVATE ROS2)
  rclcpp_components_register_node(cras_septentrio_process PLUGIN "cras::SeptentrioProcess" EXECUTABLE process)
  add_dependencies(process cras_septentrio_process)

  install(TARGETS cras_septentrio_process
    EXPORT cras_septentrio_process
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN *.launch.xml
  )

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN *ros2.yaml PATTERN *sbf_commands*
  )

  install(FILES
    80-cras-septentrio-gps.rules
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_copyright ament_cmake_uncrustify)
    ament_lint_auto_find_test_dependencies()
    # ament_cpplint(MAX_LINE_LENGTH 120 FILTERS "-readability/namespace")
  endif()

  ament_package()
else()
  message(FATAL_ERROR "Build failed because neither catkin nor ament found.")
endif()