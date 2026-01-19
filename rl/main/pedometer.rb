#
# Pedometer for LP Core (rl) - Unified Algorithm
# Lightweight Dynamic Threshold using EMA
#

class I2C
  def initialize()
    Copro.i2cinit()
  end
  def read(addr, len)
    Copro.i2cread(addr, len)
  end
  def write(addr, data)
    Copro.i2cwrite(addr, data)
  end
end

class ADXLResult
  attr_accessor :x, :y, :z
  def initialize(x, y, z)
    @x = x; @y = y; @z = z
  end
end

class ADXL
  DEV_ADDR = 0x1D
  CMD_MEASURE = [0x2D, 2]
  CMD_SOFTRESET = [0x1f, 0x52]
  CMD_FILTER = [0x2C, 0x13]
  CMD_FIFO = [0x0E]

  def initialize(i2c)
    @i2c = i2c
    @i2c.write(DEV_ADDR, CMD_SOFTRESET)
    Copro.delayMs(10)
    @i2c.write(DEV_ADDR, CMD_FILTER)
    Copro.delayMs(10)
  end

  def on()
    @i2c.write(DEV_ADDR, CMD_MEASURE)
    Copro.delayMs(50)
  end

  def conv(ary, base)
    ((ary.getbyte(base) << 26) >> 18) + ary.getbyte(base+1)
  end

  def read()
    @i2c.write(DEV_ADDR, CMD_FIFO)
    Copro.delayMs(1)
    val = @i2c.read(DEV_ADDR, 6)
    return nil if val.size == 0
    ADXLResult.new(conv(val, 0), conv(val, 2), conv(val, 4))
  end
end

# === Parameters ===
MIN_SENSITIVITY = 2000
STEP_TIMEOUT = 50
REGULATION_STEPS = 4

# === State Variables ===
step_count = 0
ema_mag = 0
dynamic_thresh = 0
max_peak = 0
min_peak = 0
looking_for_max = true
samples_since_change = 0
reg_mode = false
consec_steps = 0
gpio_state = false

# === Setup ===
Copro.gpio_output 1
Copro.gpio(1, true)
i2c = I2C.new()
acc = ADXL.new(i2c)
acc.on()
Copro.gpio(1, false)

puts "Starting Pedometer (rl) - Unified Algorithm"

# === Main Loop (LP Core) ===
Copro.sleep_and_run do
  while true do
    v = acc.read()
    if v
      # Magnitude
      mag = v.x.abs + v.y.abs + v.z.abs
      
      # EMA LPF
      if ema_mag == 0
        ema_mag = mag
      else
        ema_mag = (ema_mag * 3 + mag) / 4
      end
      
      filtered = ema_mag
      samples_since_change += 1

      if looking_for_max
        if filtered > max_peak
          max_peak = filtered
          samples_since_change = 0
        elsif samples_since_change > 5 && (max_peak - filtered) > 500
          # Max Peak Detected
          looking_for_max = false
          min_peak = filtered
          samples_since_change = 0
        end
      else
        if filtered < min_peak
          min_peak = filtered
          samples_since_change = 0
        elsif samples_since_change > 5 && (filtered - min_peak) > 500
          # Min Peak (Valley) Detected -> Step Cycle Complete
          peak_diff = max_peak - min_peak
          mid_point = (max_peak + min_peak) / 2
          
          # Update Dynamic Threshold
          if dynamic_thresh == 0
            dynamic_thresh = mid_point
          else
            dynamic_thresh = (dynamic_thresh * 3 + mid_point) / 4
          end
          
          # Validation
          if peak_diff > MIN_SENSITIVITY
            if reg_mode
              step_count += 1
              gpio_state = !gpio_state
              Copro.gpio(1, gpio_state)
              p step_count
            else
              consec_steps += 1
              if consec_steps >= REGULATION_STEPS
                reg_mode = true
                step_count += consec_steps
                consec_steps = 0
                p step_count
              end
            end
          else
            # Noise
            consec_steps = 0 if !reg_mode
          end
          
          looking_for_max = true
          max_peak = filtered
          samples_since_change = 0
        end
      end
      
      # Timeout
      if samples_since_change > STEP_TIMEOUT
        looking_for_max = true
        max_peak = filtered
        consec_steps = 0 if !reg_mode
      end
    end

    Copro.delayMs(20) # 50Hz
  end
end
