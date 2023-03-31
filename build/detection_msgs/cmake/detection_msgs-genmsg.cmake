# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "detection_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Idetection_msgs:/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(detection_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" "std_msgs/Header:detection_msgs/BoundingBox"
)

get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" "sensor_msgs/Image:geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:detection_msgs/ObjectHypothesisWithPose:detection_msgs/BoundingBox2D:geometry_msgs/Pose2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" "sensor_msgs/Image:geometry_msgs/Quaternion:geometry_msgs/Pose:detection_msgs/Detection2D:std_msgs/Header:detection_msgs/ObjectHypothesisWithPose:detection_msgs/BoundingBox2D:geometry_msgs/Pose2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_custom_target(_detection_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_msgs" "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" "geometry_msgs/Pose2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)
_generate_msg_cpp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(detection_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(detection_msgs_generate_messages detection_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_cpp _detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_msgs_gencpp)
add_dependencies(detection_msgs_gencpp detection_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)
_generate_msg_eus(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(detection_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(detection_msgs_generate_messages detection_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_eus _detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_msgs_geneus)
add_dependencies(detection_msgs_geneus detection_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)
_generate_msg_lisp(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(detection_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(detection_msgs_generate_messages detection_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_lisp _detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_msgs_genlisp)
add_dependencies(detection_msgs_genlisp detection_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)
_generate_msg_nodejs(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(detection_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(detection_msgs_generate_messages detection_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_nodejs _detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_msgs_gennodejs)
add_dependencies(detection_msgs_gennodejs detection_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg;/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)
_generate_msg_py(detection_msgs
  "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(detection_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(detection_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(detection_msgs_generate_messages detection_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBoxes.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/Detection2DArray.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/ObjectHypothesisWithPose.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/justin/robotiq_ur10_sim_ws/src/detection_msgs/msg/BoundingBox2D.msg" NAME_WE)
add_dependencies(detection_msgs_generate_messages_py _detection_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_msgs_genpy)
add_dependencies(detection_msgs_genpy detection_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(detection_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(detection_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(detection_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(detection_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(detection_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(detection_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(detection_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(detection_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(detection_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(detection_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(detection_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(detection_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(detection_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(detection_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(detection_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
