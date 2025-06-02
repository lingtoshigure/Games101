cmake_minimum_required(VERSION 2.8)

# Application source
set(APPLICATION_SOURCE
    rope.cpp
    application.cpp
    main.cpp
)


add_executable(ropesim ${APPLICATION_SOURCE})

set_target_properties(ropesim PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Binary_DIR}/Assignment8/Debug)
set_target_properties(ropesim PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Binary_DIR}/Assignment8/Release)

target_compile_definitions(ropesim PRIVATE
  GLEW_STATIC

)
if(MSVC)
    set_target_properties(ropesim PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    target_compile_definitions(ropesim PRIVATE _USE_MATH_DEFINES)
    target_compile_options(ropesim PRIVATE /MP)
    target_compile_options(ropesim PRIVATE /EHs-c- /GR-)
endif()

target_include_directories(ropesim PRIVATE
    ${ThirdParty_DIR}
    ${ThirdParty_DIR}/freetype/include
    ${ThirdParty_DIR}/glew/include
    ${ThirdParty_DIR}/glfw/include
    ${CMAKE_SOURCE_DIR}/CGL/include

)

target_link_directories(ropesim PRIVATE
    $<$<CONFIG:Debug>:${Binary_DIR}/CGL/Debug>
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/freetype/build/Debug>
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/glew/build/lib/Debug>
    $<$<CONFIG:Debug>:${ThirdParty_DIR}/glfw/build/src/Debug>

    $<$<CONFIG:Release>:${Binary_DIR}/CGL/Release>
    $<$<CONFIG:Release>:${ThirdParty_DIR}/freetype/build/Release>
    $<$<CONFIG:Release>:${ThirdParty_DIR}/glew/build/lib/Release>
    $<$<CONFIG:Release>:${ThirdParty_DIR}/glfw/build/src/Release>
    
)

target_link_libraries(ropesim PRIVATE
    $<$<CONFIG:Debug>:CGLd>
    $<$<CONFIG:Debug>:freetyped>
    $<$<CONFIG:Debug>:libglew32d>
    $<$<CONFIG:Debug>:glfw3>
    $<$<CONFIG:Debug>:opengl32>
    
    $<$<CONFIG:Release>:CGL>
    $<$<CONFIG:Release>:freetype>
    $<$<CONFIG:Release>:libglew32>
    $<$<CONFIG:Release>:glfw3>
    $<$<CONFIG:Release>:opengl32>
)

add_dependencies(ropesim CGL)