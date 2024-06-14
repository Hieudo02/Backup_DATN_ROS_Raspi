<<<<<<< HEAD
# Install script for directory: /home/pi/datn_navbot/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/datn_navbot/install")
=======
# Install script for directory: /home/pi/datn_navbot_pi/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/_setup_util.py")
=======
   "/home/pi/datn_navbot_pi/install/_setup_util.py")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE PROGRAM FILES "/home/pi/datn_navbot/build/catkin_generated/installspace/_setup_util.py")
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE PROGRAM FILES "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/_setup_util.py")
>>>>>>> Upload all file run on pi
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/env.sh")
=======
   "/home/pi/datn_navbot_pi/install/env.sh")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE PROGRAM FILES "/home/pi/datn_navbot/build/catkin_generated/installspace/env.sh")
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE PROGRAM FILES "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/env.sh")
>>>>>>> Upload all file run on pi
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/setup.bash;/home/pi/datn_navbot/install/local_setup.bash")
=======
   "/home/pi/datn_navbot_pi/install/setup.bash;/home/pi/datn_navbot_pi/install/local_setup.bash")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE FILE FILES
    "/home/pi/datn_navbot/build/catkin_generated/installspace/setup.bash"
    "/home/pi/datn_navbot/build/catkin_generated/installspace/local_setup.bash"
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE FILE FILES
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/setup.bash"
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/local_setup.bash"
>>>>>>> Upload all file run on pi
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/setup.sh;/home/pi/datn_navbot/install/local_setup.sh")
=======
   "/home/pi/datn_navbot_pi/install/setup.sh;/home/pi/datn_navbot_pi/install/local_setup.sh")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE FILE FILES
    "/home/pi/datn_navbot/build/catkin_generated/installspace/setup.sh"
    "/home/pi/datn_navbot/build/catkin_generated/installspace/local_setup.sh"
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE FILE FILES
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/setup.sh"
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/local_setup.sh"
>>>>>>> Upload all file run on pi
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/setup.zsh;/home/pi/datn_navbot/install/local_setup.zsh")
=======
   "/home/pi/datn_navbot_pi/install/setup.zsh;/home/pi/datn_navbot_pi/install/local_setup.zsh")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE FILE FILES
    "/home/pi/datn_navbot/build/catkin_generated/installspace/setup.zsh"
    "/home/pi/datn_navbot/build/catkin_generated/installspace/local_setup.zsh"
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE FILE FILES
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/setup.zsh"
    "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/local_setup.zsh"
>>>>>>> Upload all file run on pi
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
<<<<<<< HEAD
   "/home/pi/datn_navbot/install/.rosinstall")
=======
   "/home/pi/datn_navbot_pi/install/.rosinstall")
>>>>>>> Upload all file run on pi
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
<<<<<<< HEAD
file(INSTALL DESTINATION "/home/pi/datn_navbot/install" TYPE FILE FILES "/home/pi/datn_navbot/build/catkin_generated/installspace/.rosinstall")
=======
file(INSTALL DESTINATION "/home/pi/datn_navbot_pi/install" TYPE FILE FILES "/home/pi/datn_navbot_pi/build/catkin_generated/installspace/.rosinstall")
>>>>>>> Upload all file run on pi
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
<<<<<<< HEAD
  include("/home/pi/datn_navbot/build/gtest/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/mpu_6050_driver/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/rplidar_ros/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/localization_data_pub/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/robot_mapping/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/robot_model/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/navstack_pub/cmake_install.cmake")
  include("/home/pi/datn_navbot/build/robot_amcl/cmake_install.cmake")
=======
  include("/home/pi/datn_navbot_pi/build/gtest/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/mpu_6050_driver/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/rplidar_ros/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/localization_data_pub/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/robot_mapping/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/robot_model/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/navstack_pub/cmake_install.cmake")
  include("/home/pi/datn_navbot_pi/build/robot_amcl/cmake_install.cmake")
>>>>>>> Upload all file run on pi

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/pi/datn_navbot/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/pi/datn_navbot_pi/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> Upload all file run on pi
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
