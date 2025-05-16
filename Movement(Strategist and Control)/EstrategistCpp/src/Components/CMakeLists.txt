add_library(Components STATIC Communication.cpp ForceGenerator.cpp Kinematic.cpp Transform.cpp)
target_include_directories(Components
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Components PUBLIC DataTypes)