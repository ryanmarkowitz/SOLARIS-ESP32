idf_component_register(SRCS "solaris_phototransistor.c" "main.c" "solaris_ultrasonic.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc driver)
