cmake_minimum_required(VERSION 3.10)
project(RayTracing3)

set(CMAKE_CXX_STANDARD 17)


add_executable(RayTracing3 main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

set_target_properties(RayTracing3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment7/Debug)
set_target_properties(RayTracing3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment7/Release)

add_custom_command(TARGET RayTracing3 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/models ${Binary_DIR}/Assignment7/models
    COMMAND if $<CONFIG>==Debug
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110d.dll $<TARGET_FILE_DIR:RayTracing3>
    COMMAND if $<CONFIG>==Release
        ${CMAKE_COMMAND} -E copy_if_different ${ThirdParty_DIR}/opencv/bin/opencv_world4110.dll $<TARGET_FILE_DIR:RayTracing3>
    VERBATIM
)