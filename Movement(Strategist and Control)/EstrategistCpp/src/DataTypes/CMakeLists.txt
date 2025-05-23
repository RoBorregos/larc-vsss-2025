# Create a library for the DataTypes module
add_library(DataTypes STATIC Output.cpp Vector2.cpp)

# Add include directories for this module
target_include_directories(DataTypes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})