<<<<<<< HEAD
# Install script for directory: /home/pi/datn_navbot/src/robot_mapping

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/datn_navbot/install")
=======
# Install script for directory: /home/pi/datn_navbot_pi/src/robot_mapping

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/datn_navbot_pi/install")
>>>>>>> Upload all file run on pi
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pi/datn_navbot/build/robot_mapping/catkin_generated/installspace/robot_mapping.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pi/datn_navbot_pi/build/robot_mapping/catkin_generated/installspace/robot_mapping.pc")
>>>>>>> Upload all file run on pi
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_mapping/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/pi/datn_navbot/build/robot_mapping/catkin_generated/installspace/robot_mappingConfig.cmake"
    "/home/pi/datn_navbot/build/robot_mapping/catkin_generated/installspace/robot_mappingConfig-version.cmake"
=======
    "/home/pi/datn_navbot_pi/build/robot_mapping/catkin_generated/installspace/robot_mappingConfig.cmake"
    "/home/pi/datn_navbot_pi/build/robot_mapping/catkin_generated/installspace/robot_mappingConfig-version.cmake"
>>>>>>> Upload all file run on pi
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_mapping" TYPE FILE FILES "/home/pi/datn_navbot/src/robot_mapping/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_mapping" TYPE FILE FILES "/home/pi/datn_navbot_pi/src/robot_mapping/package.xml")
>>>>>>> Upload all file run on pi
endif()

