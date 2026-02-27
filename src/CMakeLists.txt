# Executables
add_executable(blur_border_constant sandbox/opencv/blur_border_constant.cpp)

# Include Dirs
target_include_directories(blur_border_constant PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link Libs
target_link_libraries(blur_border_constant ${OpenCV_LIBS})
