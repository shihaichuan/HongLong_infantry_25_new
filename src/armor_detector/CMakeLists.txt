cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # GCC编译器 - 禁用所有警告
    add_compile_options(-w)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Clang编译器 - 禁用所有警告
    add_compile_options(-w -Wno-everything)
else()
    # 其他编译器 - 尝试禁用警告
    add_compile_options(/w)
endif()

# 3. 添加宏定义，显式忽略特定警告
add_compile_definitions(
    _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    _CRT_SECURE_NO_WARNINGS
)

# 4. 禁用所有被视为错误的警告
string(REPLACE " -Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE " /WX" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")

# 5. 导出编译命令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)

# G2O配置
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
find_package(G2O)  # 改为可选

# Sophus配置
find_package(Sophus)
find_package(fmt REQUIRED)

# ROS相关依赖
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########

# 首先定义源文件
set(ARMOR_DETECTOR_SOURCES
    src/detector_node.cpp
    src/detector.cpp
    src/armor_pose_estimator.cpp
    src/graph_optimizer.cpp
    src/number_classifier.cpp
    src/light_corner_corrector.cpp
    src/ba_solver.cpp
)

# 创建库目标
ament_auto_add_library(armor_detector SHARED ${ARMOR_DETECTOR_SOURCES})

# 在目标创建后设置属性
if(TARGET armor_detector)
    # 包含目录
    target_include_directories(armor_detector PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
    )
    
    # 准备所有需要链接的库
    set(ALL_LINK_LIBRARIES
        ${OpenCV_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${image_transport_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        fmt::fmt
        TBB::tbb
    )
    
    # 可选的G2O支持
    if(G2O_FOUND)
        target_include_directories(armor_detector PUBLIC ${G2O_INCLUDE_DIRS})
        list(APPEND ALL_LINK_LIBRARIES
            g2o_core
            g2o_stuff
            g2o_solver_csparse
            g2o_solver_dense
            g2o_csparse_extension
            g2o_types_sba
            g2o_types_slam3d
        )
    else()
        message(WARNING "G2O not found, skipping G2O integration")
    endif()
    
    # 可选的Sophus支持
    if(Sophus_FOUND)
        target_include_directories(armor_detector PUBLIC ${Sophus_INCLUDE_DIRS})
        list(APPEND ALL_LINK_LIBRARIES Sophus::Sophus)
    else()
        message(WARNING "Sophus not found, skipping Sophus integration")
    endif()
    
    # 架构特定的库路径
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        target_link_directories(armor_detector PUBLIC lib/amd64)
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        target_link_directories(armor_detector PUBLIC lib/arm64)
    else()
        message(WARNING "Unsupported host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
    endif()
    
    # MindVision SDK
    if(EXISTS "/opt/MVS/lib")
        target_link_directories(armor_detector PUBLIC /opt/MVS/lib)
        list(APPEND ALL_LINK_LIBRARIES MVSDK)
    endif()
    
    # 合并所有链接库到一个调用中（使用不带关键字的形式）
    target_link_libraries(armor_detector ${ALL_LINK_LIBRARIES})
    
    # 注册节点组件
    rclcpp_components_register_node(armor_detector
        PLUGIN "rm_auto_aim::ArmorDetectorNode"
        EXECUTABLE armor_detector_node
    )
endif()

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
endif()

#############
## Install ##
#############

# 自动处理安装规则
ament_auto_package(
    INSTALL_TO_SHARE
    docs
    model

)

# 安装头文件
install(DIRECTORY include/ DESTINATION include)