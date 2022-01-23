cmake_minimum_required(VERSION 3.13.4)

idf_component_register(
        SRCS
        test_SSD1306.c
        test_SSD1306_i2c.c
        u8g2_hal_example_main.c
        INCLUDE_DIRS .
)
