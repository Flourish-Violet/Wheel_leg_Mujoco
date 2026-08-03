cmake_minimum_required(VERSION 3.20)
project(wheel_leg_cpp_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MUJOCO_ROOT "D:/APP/Mujoco")

add_executable(wheel_leg_cpp_control
    src/main.cpp
    src/viewer_glfw.cpp
    src/Algorithm/vmc.cpp
)

add_library(vmc_bridge SHARED
    src/Algorithm/vmc.cpp
)

set_target_properties(vmc_bridge PROPERTIES
    PREFIX ""
    OUTPUT_NAME "vmc_bridge"
)

target_include_directories(wheel_leg_cpp_control PRIVATE
    "${MUJOCO_ROOT}/include"
)

target_include_directories(vmc_bridge PRIVATE
    "${MUJOCO_ROOT}/include"
)

if (MSVC)
    target_link_libraries(wheel_leg_cpp_control PRIVATE
        "${MUJOCO_ROOT}/lib/mujoco.lib"
    )
else()
    target_link_libraries(wheel_leg_cpp_control PRIVATE
        "${MUJOCO_ROOT}/bin/mujoco.dll"
    )
endif()

add_custom_command(TARGET wheel_leg_cpp_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_ROOT}/bin/mujoco.dll"
        $<TARGET_FILE_DIR:wheel_leg_cpp_control>
)

set(GLFW_DLL "D:/APP/Mujoco/MujocoRef/wheel_legged_robot_sim/.venv/Lib/site-packages/glfw/glfw3.dll")
if (EXISTS "${GLFW_DLL}")
    add_custom_command(TARGET wheel_leg_cpp_control POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLFW_DLL}"
            $<TARGET_FILE_DIR:wheel_leg_cpp_control>
    )
endif()
