idf_component_register(
  SRCS main.c mpu6050_dmp.cpp
  INCLUDE_DIRS .
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wall -Wextra -Wshadow)
