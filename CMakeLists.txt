cmake_minimum_required(VERSION 2.8.3)
project(dumbo_contact_point_estimation)

find_package(catkin REQUIRED COMPONENTS roscpp contact_point_estimation cart_traj_generators geometry_msgs dumbo_cart_vel_controller std_srvs eigen_conversions kdl_conversions)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS roscpp contact_point_estimation cart_traj_generators geometry_msgs dumbo_cart_vel_controller std_srvs eigen_conversions kdl_conversions
  LIBRARIES 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(dumbo_surface_tracing_controller_node src/dumbo_surface_tracing_controller_node.cpp)
target_link_libraries(dumbo_surface_tracing_controller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS dumbo_surface_tracing_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

