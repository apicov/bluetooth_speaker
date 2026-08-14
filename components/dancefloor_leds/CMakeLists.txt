idf_component_register(SRCS "beat_detect.c" "analysis.cpp" "analysers.cpp"
                            "resample.c" "ml_arena.c" "ml_lane.cpp" "analyser_tflm.cpp"
                            "patterns.cpp" "visualiser.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES led_strip_wrapper esp_driver_gpio esp_timer)

#
# esp-tflite-micro, for analyser_tflm.cpp -- and it must be OPTIONAL, three
# times over.
#
# The whole of analyser_tflm.cpp is inside `#if CONFIG_DANCEFLOOR_ML_TFLM`, so
# with the setting off it compiles to nothing and the component is not fetched at
# all. Naming it in REQUIRES above would then fail the hub, the bridge and the
# classic satellite on a component that does not exist.
#
# Nor can the REQUIRES list be made conditional on CONFIG_DANCEFLOOR_ML_TFLM:
# component requirements are expanded EARLY, before sdkconfig is available, so
# the `if()` is simply false on every build including the one that needs it. That
# reads as a fetch that succeeded followed by a missing micro_interpreter.h,
# which is how this comment came to be written.
#
# idf_component_optional_requires() runs after registration, where config IS
# available, and links only if the component is actually in the build -- both
# conditions this needs. The dependency that puts it there is declared in
# satellite/main/idf_component.yml; the note in this component's own
# idf_component.yml says why it cannot live beside the code it belongs to.
#
if(CONFIG_DANCEFLOOR_ML_TFLM)
    idf_component_optional_requires(PUBLIC espressif__esp-tflite-micro)
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>)
