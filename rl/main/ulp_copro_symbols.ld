/* ULP copro function symbols
 * These symbols map to the actual function addresses in the ULP binary.
 * Addresses are taken from build/esp-idf/main/ulp_main/ulp_main.sym
 * NOTE: If ULP code changes, these addresses may need to be updated.
 */
ulp_copro_GPIOpulseIn = 0x5000060c;
ulp_copro_GPIOget = 0x500006ea;
ulp_copro_GPIOset = 0x50000706;
ulp_copro_delayMs = 0x50000602;
ulp_copro_delayUs = 0x5000059c;
ulp_copro_I2Cread = 0x50000722;
ulp_copro_I2Cwrite = 0x50000766;
