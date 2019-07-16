cmake_minimum_required(VERSION 2.8.3)
project(cloud_proc)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        message_runtime
        nav_msgs
        rospy
        sensor_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_msgs
        rospy
        sensor_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
)

catkin_install_python(
    PROGRAMS
        scripts/box_filter
        scripts/cloud_proc
        scripts/odom_cloud_transformer
        scripts/odom_tf
        scripts/tf_odom
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
