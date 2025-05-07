cmake_minimum_required(VERSION 3.10.0)
project(EstrategistF VERSION 0.1.0 LANGUAGES C CXX)

add_executable(EstrategistF main.cpp)
add_subdirectory(src/Components)
add_subdirectory(src/DataTypes)
add_subdirectory(src/Entities)



target_link_libraries(EstrategistF
PUBLIC
    Components
    DataTypes
    Entities
)


