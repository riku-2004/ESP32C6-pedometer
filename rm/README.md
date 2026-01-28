# RM (Ruby on Main) Project Troubleshooting Log

## 2025/01/21 Troubleshooting Session

### 1. Bytecode Version Mismatch
- **Issue**: Runtime error `Bytecode version mismatch`.
- **Cause**: WSL default `mrbc` (from mruby 3.0.0) generates RITE 0200 format, but the project's mruby/c runtime expects RITE 0300.
- **Solution**: Built mruby 3.4.0 in WSL from source. Used the newly built `mrbc` to compile `.rb` files.
- **Command**:
  ```bash
  /path/to/mruby/build/host/bin/mrbc -Bmrbbuf -o pedometer.c pedometer.rb
  ```

### 2. Linker Error: Undefined I2C Functions
- **Issue**: Linker error `undefined reference to 'ulp_lp_core_i2c_master_...'` when building `rm`.
- **Cause**: `rm` (Main Processor) links `copro.c` which references LP Core specific I2C functions.
- **Solution**: Implemented wrapper functions in `rm/main/main.c` using standard ESP-IDF I2C driver (`driver/i2c_master.h`) to bridge the calls.

### 3. Debug Output Hang (Light Sleep Issue)
- **Issue**: Program appeared to hang or output stopped during I2C operations.
- **Cause**: `Copro.delayMs` implementation in `copro.c` was calling `esp_light_sleep_start` for ESP32-C6 target. Light Sleep stops the CPU and UART, cutting off debug output.
- **Solution**: Modified `rm/components/mruby/hal/esp32/copro/copro.c` to disable Light Sleep (`DELAY_MS_LIGHTSLEEP 0`) and use `vTaskDelay` instead.

### 4. Ruby Runtime Error (Type Error)
- **Issue**: Runtime error `undefined local variable or method '|' for String`.
- **Cause**: In mruby/c, accessing binary data string with `[]` returns a String (character) instead of an Integer (byte value), causing bitwise operations to fail.
- **Solution**: Updated `pedometer.rb` to use `getbyte(index)` instead of `[index]`.

### 5. Residual LP Core Logs
- **Issue**: Logs from a previously flashed LP Core program (`cl`) were appearing, confusing debugging.
- **Solution**: Added distinct startup banner to `rm` to clearly distinguish its output. (Eventually, overwriting LP Core binary stops the old program).
