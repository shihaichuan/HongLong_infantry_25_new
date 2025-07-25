cmake_minimum_required(VERSION 3.10)
project(aimer)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# find_package(std_msgs REQUIRED)
##################
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
# ###################
# find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::Aimer
  EXECUTABLE ${PROJECT_NAME}_node
)

# 在 CMakeLists.txt 中添加以下内容
# ament_target_dependencies(armor_solver
# base_interfaces
#   base_interfaces
# )

# 如果需要，也可以显式地添加头文件路径
# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)
# target_link_libraries(armor_solver
#     g2o_core
#     g2o_stuff
#     g2o_solver_dense
# )
# target_link_libraries(armor_solver ${g2o_LIBRARIES})
#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

endif()

#############
## Install ##
#############

ament_auto_package()