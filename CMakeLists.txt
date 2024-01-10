# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(cras_septentrio_gnss_driver)

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

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cras_septentrio_gnss_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

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