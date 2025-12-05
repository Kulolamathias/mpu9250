idf_component_register(
    SRCS 
        "main.c"
        "mpu9250.c"
        "mpu9250_example.c"
    
    REQUIRES
    PRIV_REQUIRES                    
    INCLUDE_DIRS "."
)
