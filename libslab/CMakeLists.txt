cmake_minimum_required(VERSION 2.8.12)

project(slab)

add_library(${CMAKE_PROJECT_NAME} SHARED src/slab.c)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC include)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -fPIC -Wall -Wextra -Wpedantic -Wshadow)

file(GLOB headers include/*.h)
add_custom_command(
  OUTPUT lib${CMAKE_PROJECT_NAME}_py.py
  COMMAND ctypesgen -l lib${CMAKE_PROJECT_NAME}.so --output=lib${CMAKE_PROJECT_NAME}_py.py ${headers} 2> ctypesgen.err
  DEPENDS ${headers}
  COMMENT "Generated python message definitions"
  VERBATIM
)
add_custom_target(lib${CMAKE_PROJECT_NAME}_py ALL DEPENDS lib${CMAKE_PROJECT_NAME}_py.py)
