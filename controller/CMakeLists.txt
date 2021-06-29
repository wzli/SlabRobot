cmake_minimum_required(VERSION 2.8.3)
project(slab_controller)

add_library(slab_controller SHARED src/slab_controller.c)
target_include_directories(slab_controller PUBLIC include)
target_compile_options(slab_controller PUBLIC -fPIC -Wall -Wextra -Wpedantic -Wshadow)
