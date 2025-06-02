cmake_minimum_required(VERSION 3.10)
project(Rasterizer)
set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer2 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)

target_include_directories(Rasterizer2 PRIVATE
    ${ThirdParty_DIR}
    ${ThirdParty_DIR}/eigen3
    ${ThirdParty_DIR}/opencv/include
)

target_link_libraries(Rasterizer2 
    debug ${ThirdParty_DIR}/opencv/lib/opencv_world4110d.lib
    optimized ${ThirdParty_DIR}/opencv/lib/opencv_world4110.lib
)

set_target_properties(Rasterizer2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment2/Debug)
set_target_properties(Rasterizer2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment2/Release)

add_custom_command(TARGET Rasterizer2 POST_BUILD
    COMMAND if $<CONFIG>==Debug
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110d.dll $<TARGET_FILE_DIR:Rasterizer2>
    COMMAND if $<CONFIG>==Release
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110.dll $<TARGET_FILE_DIR:Rasterizer2>
    VERBATIM
)
