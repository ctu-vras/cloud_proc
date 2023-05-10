cmake_minimum_required(VERSION 3.10.2)
project(cloud_proc)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  cras_cpp_common
  nodelet
  pcl_ros
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES improved_pcl_filter pcl_crop_box_fixed
  CATKIN_DEPENDS nodelet pcl_ros
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(pcl_crop_box_fixed src/PclCropBoxFixed.cpp)
target_link_libraries(pcl_crop_box_fixed ${catkin_LIBRARIES})
add_dependencies(pcl_crop_box_fixed ${catkin_EXPORTED_TARGETS})

add_library(improved_pcl_filter src/ImprovedPclFilter.cpp)
target_link_libraries(improved_pcl_filter ${catkin_LIBRARIES})
add_dependencies(improved_pcl_filter ${catkin_EXPORTED_TARGETS})

add_library(improved_pcl_filters src/ImprovedPclFilters.cpp)
target_link_libraries(improved_pcl_filters improved_pcl_filter pcl_crop_box_fixed ${catkin_LIBRARIES})
add_dependencies(improved_pcl_filters ${catkin_EXPORTED_TARGETS})
class_loader_hide_library_symbols(improved_pcl_filters)

add_library(
    cloud_proc_nodelets
        src/box_filter_nodelet.cpp
        src/projection_nodelet.cpp
        src/range_filter_nodelet.cpp
        src/ortho_stats_nodelet.cpp
        src/voxel_filter_nodelet.cpp
)
add_dependencies(cloud_proc_nodelets ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloud_proc_nodelets ${catkin_LIBRARIES})

catkin_install_python(
    PROGRAMS
        scripts/box_filter
        scripts/cloud_assembler
        scripts/cloud_proc
        scripts/cloud_to_image
        scripts/odom_cloud_transformer
        scripts/odom_tf
        scripts/tf_odom
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS improved_pcl_filter improved_pcl_filters pcl_crop_box_fixed cloud_proc_nodelets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES cloud_proc_nodelets.xml nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING REGEX ".*\\.hp?p?$"
)
