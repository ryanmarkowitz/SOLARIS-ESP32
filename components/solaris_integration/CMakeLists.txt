idf_component_register(SRCS "solaris_icm20948.c"
                            "solaris_ina228.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                                freertos
                                log 
                                )