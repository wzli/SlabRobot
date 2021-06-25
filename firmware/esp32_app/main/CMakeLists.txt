idf_component_register(
  SRCS main.c odrive_can.c mpu6050_dmp.cpp
  INCLUDE_DIRS .
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wall -Wextra -Wshadow)
