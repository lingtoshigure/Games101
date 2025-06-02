cmake_minimum_required(VERSION 3.10)
project(Rasterizer)
set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

target_include_directories(Rasterizer PRIVATE
    ${ThirdParty_DIR}
    ${ThirdParty_DIR}/eigen3
    ${ThirdParty_DIR}/opencv/include
)

target_link_libraries(Rasterizer
    debug ${ThirdParty_DIR}/opencv/lib/opencv_world4110d.lib
    optimized ${ThirdParty_DIR}/opencv/lib/opencv_world4110.lib
)

set_target_properties(Rasterizer PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment1/Debug)
set_target_properties(Rasterizer PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment1/Release)

add_custom_command(TARGET Rasterizer POST_BUILD
    COMMAND if $<CONFIG>==Debug
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110d.dll $<TARGET_FILE_DIR:Rasterizer>
    COMMAND if $<CONFIG>==Release
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110.dll $<TARGET_FILE_DIR:Rasterizer>
    VERBATIM
)
