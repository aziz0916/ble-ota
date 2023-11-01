set(srcs "gatts_table_creat_demo.c"
         "gpio_wakeup.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
