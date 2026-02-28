# Executables
add_executable(blur_border_constant sandbox/opencv/blur_border_constant.cpp)
add_executable(deref_line_iterator sandbox/opencv/deref_line_iterator.cpp)
add_executable(point_types sandbox/opencv/point_types.cpp)
add_executable(mat_reference_semantics sandbox/opencv/mat_reference_semantics.cpp)
add_executable(px_access sandbox/opencv/px_access.cpp)

# Include Dirs
target_include_directories(blur_border_constant PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(deref_line_iterator PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(point_types PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(mat_reference_semantics PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(px_access PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link Libs
target_link_libraries(blur_border_constant ${OpenCV_LIBS})
target_link_libraries(deref_line_iterator ${OpenCV_LIBS})
target_link_libraries(point_types ${OpenCV_LIBS})
target_link_libraries(mat_reference_semantics ${OpenCV_LIBS})
target_link_libraries(px_access ${OpenCV_LIBS})
