idf_component_register(
    SRCS "src/hal_rs485.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver hal_uart
)