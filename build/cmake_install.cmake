# Install script for directory: /home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/.catkin")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE FILE FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/.catkin")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/_setup_util.py")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE PROGRAM FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/_setup_util.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/env.sh")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE PROGRAM FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/env.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/setup.bash")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE FILE FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/setup.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/setup.sh")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE FILE FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/setup.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/setup.zsh")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE FILE FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/setup.zsh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install/.rosinstall")
FILE(INSTALL DESTINATION "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/install" TYPE FILE FILES "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/catkin_generated/installspace/.rosinstall")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/hydro/share/catkin/cmake/env-hooks/05.catkin_make.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/hydro/share/catkin/cmake/env-hooks/05.catkin_make_isolated.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/gtest/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_2dnav/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_driver/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_node/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_robot/neato_robot/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/in_class_code_day3/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/my_pf/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/neato_simulator/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/occupancygrid_mapping/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/gscam/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/beginner_tutorials/cmake_install.cmake")
  INCLUDE("/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/warmup_project/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/nlintz/Documents/Olin/2014-2015/Fall/CompRobo/comprobo2014/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
