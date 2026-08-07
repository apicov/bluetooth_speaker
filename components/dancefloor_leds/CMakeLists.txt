idf_component_register(SRCS "beat_detect.c" "analysis.cpp" "analysers.cpp"
                            "patterns.cpp" "visualiser.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES led_strip_wrapper esp_driver_gpio esp_timer)

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>)
