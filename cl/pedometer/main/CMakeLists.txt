idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ulp)
ulp_embed_binary(ulp_core_main "ulp/main.c ../../sleep.S" "main.c")
