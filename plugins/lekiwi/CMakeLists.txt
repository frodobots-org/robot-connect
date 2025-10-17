cmake_minimum_required(VERSION 3.15)

project(lekiwi)

include(FindPkgConfig)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)

include_directories(
  ${GST_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../include
)

link_directories(
  ${GST_LIBRARIES}
)

add_library(teleop_ctrl_zmq SHARED teleop_ctrl_zmq.cc)
add_library(teleop_media_gst SHARED teleop_media_gst.cc)
target_link_libraries(teleop_media_gst ${GST_LIBRARIES})
target_link_libraries(teleop_ctrl_zmq zmq)
