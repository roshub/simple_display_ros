cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(display
  SRCS display.cpp ${ROS_LIB_DIR}/time.cpp Adafruit_GFX/Adafruit_GFX.cpp Adafruit_GFX/glcdfont.c RGB-matrix-Panel-master/RGBmatrixPanel.cpp Time/Time.cpp
  HDRS Adafruit_GFX/Adafruit_GFX.h RGB-matrix-Panel-master/RGBmatrixPanel.h Time/Time.h
  BOARD mega2560
  PORT /dev/ttyACM.mega
)

