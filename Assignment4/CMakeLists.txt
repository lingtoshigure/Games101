cmake_minimum_required(VERSION 3.10)
project(BezierCurve)
set(CMAKE_CXX_STANDARD 14)

add_executable(BezierCurve main.cpp)

set_target_properties(BezierCurve PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment4/Debug)
set_target_properties(BezierCurve PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment4/Release)

target_include_directories(BezierCurve PRIVATE
    ${ThirdParty_DIR}
    ${ThirdParty_DIR}/opencv/include
)

target_link_libraries(BezierCurve
    debug ${ThirdParty_DIR}/opencv/lib/opencv_world4110d.lib
    optimized ${ThirdParty_DIR}/opencv/lib/opencv_world4110.lib
)

add_custom_command(TARGET BezierCurve POST_BUILD
    COMMAND if $<CONFIG>==Debug
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110d.dll $<TARGET_FILE_DIR:BezierCurve>
    COMMAND if $<CONFIG>==Release
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110.dll $<TARGET_FILE_DIR:BezierCurve>
    VERBATIM
)



