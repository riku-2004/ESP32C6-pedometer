idf_component_register(SRCS  "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES ulp driver mruby)
include(mrubycopro-runtime/CMakeLists.txt)
set(ulp_exp_dep_srcs "main.c")
set(ulp_app_name ulp_${COMPONENT_NAME})

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")