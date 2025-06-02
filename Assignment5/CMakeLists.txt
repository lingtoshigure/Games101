cmake_minimum_required(VERSION 3.10)
project(RayTracing)
set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)
set_target_properties(RayTracing PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment5/Debug)
set_target_properties(RayTracing PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment5/Release)
target_compile_features(RayTracing PUBLIC cxx_std_17)
target_link_libraries(RayTracing PUBLIC -fsanitize=undefined)
