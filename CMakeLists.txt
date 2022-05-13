cmake_minimum_required(VERSION 2.8.3)
project(gaussian_mixture_model)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  cmake_modules
  rosbag
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
    FILES
    Gaussian.msg
    GaussianMixture.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gaussian_mixture_model
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

###########
## Build ##
###########

include_directories(

  include
  ${catkin_INCLUDE_DIRS}
	~/Kuka_iiwa/src/my_iiwa_pkg/include
)

add_library(gaussian_mixture_model
  src/gmm.cpp
  )

add_executable(gaussian_mixture_model_node src/gmm_node.cpp)

add_dependencies(gaussian_mixture_model_node gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model
  ${catkin_LIBRARIES}
  )

target_link_libraries(gaussian_mixture_model_node
  gaussian_mixture_model
  ${catkin_LIBRARIES}
  )

add_executable(gaussian_mixture_model_rviz_converter src/gmm_rviz_converter.cpp)

add_dependencies(gaussian_mixture_model_rviz_converter gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model_rviz_converter
  ${catkin_LIBRARIES}
  )

add_executable(gaussian_mixture_model_trajectory_converter src/trajectory_converter.cpp)

add_dependencies(gaussian_mixture_model_trajectory_converter gaussian_mixture_model_generate_messages_cpp)

target_link_libraries(gaussian_mixture_model_trajectory_converter
  ${catkin_LIBRARIES}
  )

## Added by zizowasfy
add_executable(dtw_test src/dtw_test.cpp)

add_executable(Data_Acq src/data_acq.cpp)
target_link_libraries(Data_Acq ${catkin_LIBRARIES})

add_executable(demons_DTW src/demons_DTW.cpp)
target_link_libraries(demons_DTW ${catkin_LIBRARIES})
## \Added by zizowasfy


#############
## Install ##
#############
