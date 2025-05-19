# Custom Zephyr function that imports the mlx90640/ subdirectory if 
# the Kconfig option MLX90640 is defined
add_subdirectory_ifdef(CONFIG_MLX90640 sensor/mlx90640)