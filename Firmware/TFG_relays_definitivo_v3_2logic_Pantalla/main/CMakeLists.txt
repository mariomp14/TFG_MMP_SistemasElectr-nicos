# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS "main.c"
         "my_wifi/my_wifi.c"
         "my_mqtt/my_mqtt.c"
    INCLUDE_DIRS "."
                 "my_wifi/include"
                 "my_mqtt/include"
)
