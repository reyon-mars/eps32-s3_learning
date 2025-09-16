idf_component_register(
    SRCS "src/rs485.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver uart esp-modbus
)
