CMAKE_MINIMUM_REQUIRED(VERSION 3.13)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(BackwardEuler)

set(SOURCE_FILES
  src/main.cpp
  src/particle_system.cpp
  src/mass_spring.cpp
  src/integrator.cpp
  src/spring.cpp
  src/vec3.cpp
  src/clock.cpp
  src/contact.cpp
)

set(RENDERER_DIR ${CMAKE_SOURCE_DIR}/../renderer/)

add_subdirectory(${RENDERER_DIR} ${CMAKE_SOURCE_DIR}/build/render)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wall)
# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -g)
# set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -O3)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})


# External libraries
find_package(Eigen3 REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${EIGEN_INCLUDE_DIR}
  ${RENDERER_DIR}/include/
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  renderer
  ${EIGEN_LIBRARIES}
)
