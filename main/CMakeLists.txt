idf_component_register(
    SRCS "hello_world_main.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash driver
    PRIV_REQUIRES spi_flash
)
