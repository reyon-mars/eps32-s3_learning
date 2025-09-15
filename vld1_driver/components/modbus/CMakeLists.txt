idf_component_register(
    SRCS "src/modbus_forwarder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES hal_rs485 
)