cmake_minimum_required(VERSION 2.8.12)

project(slab_controller)

add_library(${CMAKE_PROJECT_NAME} SHARED src/slab_controller.c)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -fPIC -Wall -Wextra -Wpedantic -Wshadow)

file(GLOB headers include/*.h)
add_custom_command(
  OUTPUT slab_ctypes.py
  COMMAND ctypesgen -l lib${CMAKE_PROJECT_NAME}.so --output=slab_ctypes.py ${headers}
  DEPENDS ${CMAKE_PROJECT_NAME}
  COMMENT "Generate python message definitions"
  VERBATIM
)
add_custom_target(slab_ctypes ALL DEPENDS slab_ctypes.py)
