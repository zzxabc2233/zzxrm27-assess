cmake_minimum_required(VERSION 3.16)

# 更灵活地查找OpenVINO
find_package(OpenVINO QUIET)
if(NOT OpenVINO_FOUND)
    # 尝试常见的安装路径
    set(OPENVINO_POSSIBLE_PATHS
        "/usr/lib/cmake/openvino2024.6.0"
        "/opt/intel/openvino/runtime/cmake"
        "/opt/intel/openvino_2024/runtime/cmake"
        "/opt/intel/openvino_2024.6.0/runtime/cmake"
        "/usr/local/runtime/cmake"
    )
    foreach(PATH ${OPENVINO_POSSIBLE_PATHS})
        if(EXISTS "${PATH}")
            set(OpenVINO_DIR "${PATH}")
            message(STATUS "尝试使用OpenVINO路径: ${PATH}")
            find_package(OpenVINO QUIET PATHS "${PATH}")
            if(OpenVINO_FOUND)
                break()
            endif()
        endif()
    endforeach()
    if(NOT OpenVINO_FOUND)
        message(FATAL_ERROR "未找到OpenVINO！请安装OpenVINO或设置OpenVINO_DIR环境变量")
    endif()
endif()

# Ceres设为可选
find_package(Ceres QUIET)
if(NOT Ceres_FOUND)
    find_package(Ceres QUIET PATHS /usr/local/lib/cmake/Ceres)
endif()

add_library(auto_buff OBJECT 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff auto_aim openvino::runtime)
if(Ceres_FOUND)
    target_link_libraries(auto_buff ${CERES_LIBRARIES})
endif()