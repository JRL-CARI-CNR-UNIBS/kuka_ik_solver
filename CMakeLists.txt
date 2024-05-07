cmake_minimum_required(VERSION 3.8)
project(kuka_ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_TESTING "Tests" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ik_solver REQUIRED)
find_package(opw_kinematics REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/kuka_ik_solver.cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  ik_solver)
target_link_libraries(${PROJECT_NAME} PUBLIC
  opw_kinematics::opw_kinematics)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

pluginlib_export_plugin_description_file(ik_solver plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} export_${PROJECT_NAME})
ament_export_dependencies(${PROJECT_NAME} pluginlib ik_solver opw_kinematics)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
#  ament_target_dependencies(${PROJECT_NAME}_test
#    std_msgs
#  )
  target_link_libraries(${PROJECT_NAME}_test opw_kinematics::opw_kinematics ${PROJECT_NAME})
endif()

ament_package()
