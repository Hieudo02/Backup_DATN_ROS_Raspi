<<<<<<< HEAD
# Install script for directory: /home/pi/datn_navbot/src/navstack_pub

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/datn_navbot/install")
=======
# Install script for directory: /home/pi/datn_navbot_pi/src/navstack_pub

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pi/datn_navbot/build/navstack_pub/catkin_generated/installspace/navstack_pub.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pi/datn_navbot_pi/build/navstack_pub/catkin_generated/installspace/navstack_pub.pc")
>>>>>>> Upload all file run on pi
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navstack_pub/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/pi/datn_navbot/build/navstack_pub/catkin_generated/installspace/navstack_pubConfig.cmake"
    "/home/pi/datn_navbot/build/navstack_pub/catkin_generated/installspace/navstack_pubConfig-version.cmake"
=======
    "/home/pi/datn_navbot_pi/build/navstack_pub/catkin_generated/installspace/navstack_pubConfig.cmake"
    "/home/pi/datn_navbot_pi/build/navstack_pub/catkin_generated/installspace/navstack_pubConfig-version.cmake"
>>>>>>> Upload all file run on pi
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navstack_pub" TYPE FILE FILES "/home/pi/datn_navbot/src/navstack_pub/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/navstack_pub" TYPE FILE FILES "/home/pi/datn_navbot_pi/src/navstack_pub/package.xml")
>>>>>>> Upload all file run on pi
endif()

