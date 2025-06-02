cmake_minimum_required(VERSION 3.10)
project(CGL)

set(CMAKE_CXX_STANDARD 17)

set(currentPath "${CMAKE_SOURCE_DIR}/CGL")

#收集所有源文件
file(GLOB_RECURSE SOURCE_FILES "${currentPath}/*")

add_library(CGL STATIC ${SOURCE_FILES})

set_target_properties(CGL PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin/CGL/Debug"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin/CGL/Release"
    DEBUG_POSTFIX "d"
)
target_compile_definitions(CGL PRIVATE GLEW_STATIC)

if(MSVC)
    set_target_properties(CGL PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_compile_definitions(CGL PRIVATE _USE_MATH_DEFINES)
    target_compile_options(CGL PRIVATE /MP)
    target_compile_options(CGL PRIVATE /EHs-c- /GR-)
endif()

target_include_directories(CGL PRIVATE
    ${ThirdParty_DIR}
    ${ThirdParty_DIR}/freetype/include
    ${ThirdParty_DIR}/glew/include
    ${ThirdParty_DIR}/glfw/include
)


target_link_directories(CGL PRIVATE
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/freetype/build/Debug>
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/glew/build/lib/Debug>
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/glfw/build/src/Debug>

    $<$<CONFIG:Release>:${ThirdParty_DIR}/freetype/build/Release>
    $<$<CONFIG:Release>:${ThirdParty_DIR}/glew/build/lib/Release>
    $<$<CONFIG:Release>:${ThirdParty_DIR}/glfw/build/src/Release>
    
)

target_link_libraries(CGL PRIVATE
    $<$<CONFIG:Debug>:freetyped>
    $<$<CONFIG:Debug>:libglew32d>
    $<$<CONFIG:Debug>:glfw3>
    $<$<CONFIG:Debug>:opengl32>
    
    $<$<CONFIG:Release>:freetype>
    $<$<CONFIG:Release>:libglew32>
    $<$<CONFIG:Release>:glfw3>
    $<$<CONFIG:Release>:opengl32>
)