# Executables
add_executable(test_local_occupancy_mask scan/test_local_occupancy_mask.cpp)

# Include Dirs
target_include_directories(test_local_occupancy_mask PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link Libs
target_link_libraries(test_local_occupancy_mask ${OpenCV_LIBS})
