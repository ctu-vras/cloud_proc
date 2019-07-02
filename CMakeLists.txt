cmake_minimum_required(VERSION 2.8.3)
project(cloud_proc)

find_package(catkin REQUIRED
    COMPONENTS message_runtime nav_msgs rospy sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS message_runtime nav_msgs rospy sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

catkin_install_python(
    PROGRAMS scripts/box_filter scripts/cloud_proc scripts/tf_odom
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
