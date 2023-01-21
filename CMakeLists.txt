CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED true)

project(BackwardEuler)

set(SOURCE_FILES
  src/main.cpp
  src/system.cpp
  src/spring.cpp
  src/vec3.cpp
  src/spring_list.cpp
  src/clock.cpp
  src/mesh.cpp
)
set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -g)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# External libraries
find_package(raylib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${RAYLIB_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${ASSIMP_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  raylib
  rt
  ${RAYLIB_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)
