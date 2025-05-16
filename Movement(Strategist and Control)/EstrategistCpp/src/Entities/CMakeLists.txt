# Create a library for the Entities module
add_library(Entities STATIC Ball.cpp Robot.cpp Entity.cpp Line.cpp)

# Add include directories for this module
target_include_directories(Entities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Entities PUBLIC Components)