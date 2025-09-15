idf_component_register(
    SRCS "src/vld1.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos hal_uart
)