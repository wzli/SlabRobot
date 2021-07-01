idf_component_register(
        SRCS
          "esp32-ps3/src/ps3.c"
          "esp32-ps3/src/ps3_spp.c"
          "esp32-ps3/src/ps3_parser.c"
          "esp32-ps3/src/ps3_l2cap.c"
          #"esp32-ps3/src/ps3_gap.c"
        REQUIRES bt
        INCLUDE_DIRS esp32-ps3/src/include
        PRIV_INCLUDE_DIRS
          $ENV{IDF_PATH}/components/bt/common/include
          $ENV{IDF_PATH}/components/bt/host/bluedroid/common/include
          $ENV{IDF_PATH}/components/bt/host/bluedroid/stack/include
          $ENV{IDF_PATH}/components/bt/host/bluedroid/stack/l2cap/include
          #$ENV{IDF_PATH}/components/bt/host/bluedroid/stack/gap/include
)
#target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_IDF_COMPATIBILITY=3)
