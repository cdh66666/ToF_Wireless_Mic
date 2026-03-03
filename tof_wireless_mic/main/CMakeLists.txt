# CMakeLists for the main application component
# This file was missing which caused the IDF build to perform a "minimal build".

idf_component_register(
    SRCS "main.c"
         "audio_wifi.c"
         "ToF.c"
         "ws2812.c"
    INCLUDE_DIRS "."
)
