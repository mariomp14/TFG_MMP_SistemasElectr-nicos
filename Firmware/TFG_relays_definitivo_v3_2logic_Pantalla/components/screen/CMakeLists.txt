idf_component_register(SRCS "screen.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver freertos lvgl esp_lcd atanisoft__esp_lcd_touch_xpt2046 atanisoft__esp_lcd_ili9488 ui)

