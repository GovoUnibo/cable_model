cmake_minimum_required(VERSION 3.0.2)
project(cable_model_pkg)

set(CMAKE_BUILD_TYPE Release)
# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(gazebo REQUIRED) # prima questo un Q&A su gazebo, se no genera errore

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_plugins
  gazebo_ros
  message_generation
  std_msgs
  genmsg
  # gazebo 
  Eigen3
  
)





include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/cable_model
  

)

# add_message_files(DIRECTORY msg
#   FILES
#   coordinates.msg
# )

add_service_files(DIRECTORY srv
  FILES
  GraspMsg.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs
)


# COMMENTED OUT - Original world plugin
# add_library(cable_push SHARED src/WorldCablePlugin.cpp  src/WorldCablePlugin.hpp
#                         include/cable_model/sdf_builder.cpp       include/cable_model/sdf_builder.hpp
#                         include/cable_model/sdf_sphere.cpp       include/cable_model/sdf_sphere.hpp
#                         include/cable_model/sdf_cylinder.cpp     include/cable_model/sdf_cylinder.hpp
#                   )
# target_link_libraries(cable_push PUBLIC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# COMMENTED OUT - Original model plugin (full version)
# add_library(model_push SHARED src/ModelCablePlugin.cpp src/ModelCablePlugin.hpp
#                               include/csv_exporter.cpp include/csv_exporter.hpp
#                               include/cable_dynamics.cpp include/cable_dynamics.hpp
#                               include/cable_model.cpp include/cable_model.hpp  
#                               include/cable_masses.cpp include/cable_masses.hpp                          
# )
# target_link_libraries(model_push PUBLIC ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
# add_dependencies(model_push ${catkin_EXPORTED_TARGETS}) #da aggiungere se si usano i messaggi, perchè altrimenti builda lo script prima del build dei messaggi


add_library(cable_push SHARED src/CablePlugin.cpp src/CablePlugin.hpp
                              include/cable_model/sdf_builder.cpp       include/cable_model/sdf_builder.hpp
                              include/cable_model/sdf_sphere.cpp       include/cable_model/sdf_sphere.hpp
                              include/cable_model/sdf_cylinder.cpp     include/cable_model/sdf_cylinder.hpp
                              include/RKDP45.cpp          include/RKDP45.hpp
                              include/cable_dynamics.cpp  include/cable_dynamics.hpp
                              include/cable_model.cpp     include/cable_model.hpp  
                              include/cable_masses.cpp    include/cable_masses.hpp   
                              )                       

target_link_libraries(cable_push PUBLIC ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(cable_push ${catkin_EXPORTED_TARGETS})



