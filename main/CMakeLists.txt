idf_component_register(
  SRCS "app_main.c"
  INCLUDE_DIRS "."
  REQUIRES app driver esp_adc mq digital_io cli
)
