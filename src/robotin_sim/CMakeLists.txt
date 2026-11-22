cmake_minimum_required(VERSION 3.16)
project(robotin_sim)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-transport REQUIRED)

add_library(robotin_plant SHARED src/plant.cpp)
target_compile_features(robotin_plant PRIVATE cxx_std_17)
target_link_libraries(robotin_plant PRIVATE gz-sim::gz-sim gz-msgs::gz-msgs
  gz-plugin::register gz-transport::gz-transport)

install(TARGETS robotin_plant LIBRARY DESTINATION lib)
install(FILES scripts/make_world.py DESTINATION share/${PROJECT_NAME}/scripts)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/sorter.py scripts/viewer.py DESTINATION lib/${PROJECT_NAME})

ament_package()
