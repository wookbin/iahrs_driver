cmake_minimum_required(VERSION 2.8.3)
project(iahrs_driver)

###################################
# Packages
###################################
find_package(catkin REQUIRED
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf
  geometry_msgs
)

## add Service....
add_service_files(
  FILES
  all_data_reset.srv
  euler_angle_init.srv
  euler_angle_reset.srv
  pose_velocity_reset.srv
  reboot_sensor.srv
)

generate_messages(   
  DEPENDENCIES  
  std_msgs
  std_srvs
  geometry_msgs
) 

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES iahrs_driver
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 ${catkin_INCLUDE_DIRS}
)

#add_compile_options(-std=c++11)
add_executable(iahrs_driver src/iahrs_driver.cpp)
add_dependencies(iahrs_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iahrs_driver ${catkin_LIBRARIES})
