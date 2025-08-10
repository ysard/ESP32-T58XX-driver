idf_component_register(
    SRCS "t58xx_aad_mode.c" "t5838_drv.c" "t5848_drv.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2s
)
