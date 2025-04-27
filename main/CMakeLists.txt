idf_component_register(SRCS "main.c"  "BNO055/bno055.c" "BNO055/nvs_handler.c" "src/bno055_handler.c" "src/LPF.c" "src/position_tracker.c"
                    INCLUDE_DIRS "." "./inc" "./BNO055/include" )
