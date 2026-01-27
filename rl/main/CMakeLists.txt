idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio ulp mruby
)

# Add custom linker script for ULP copro function symbols
target_link_options(${COMPONENT_LIB} PUBLIC "-T${CMAKE_CURRENT_SOURCE_DIR}/ulp_copro_symbols.ld")

include(mrubycopro-runtime/CMakeLists.txt)
set(ulp_exp_dep_srcs "main.c")
set(ulp_app_name ulp_${COMPONENT_NAME})

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")

