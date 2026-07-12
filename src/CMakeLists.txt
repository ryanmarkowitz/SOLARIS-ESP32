# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                    REQUIRES esp_driver_pcnt nvs_flash driver esp_driver_mcpwm esp_adc driver esp_rom freertos log esp_driver_tsens)

