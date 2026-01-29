
# Pedometer for LP Core (rl)
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
  CTRL_REG = 0x2D
  REG_DATA = 0x0E
  CMD_MEASURE = [0x2D, 2]
  CMD_STANDBY = [0x2D, 0]
  CMD_SOFTRESET = [0x1f, 0x52]
  CMD_FILTER = [0x2C, 0x13]
  CMD_FIFO = [0x18]
  def initialize(i2c)
    @i2c = i2c
    Copro.delayMs(100)
    @i2c.write(DEV_ADDR, CMD_SOFTRESET)
    # puts "After SOFTRESET: #{a}"
    # p a
    Copro.delayMs(100) #add delay
    @i2c.write(DEV_ADDR, CMD_FILTER)
    # puts "After FILTER: #{b}"
    # p b
    Copro.delayMs(100) #add delay
    @i2c.write(DEV_ADDR, CMD_MEASURE)
    # puts "After MEASURE: #{c}"
    # p c
    Copro.delayMs(100) #add delay
  end

  def on()
    @i2c.write(DEV_ADDR, CMD_MEASURE) #計測モード
  end

  def off()
    @i2c.write(DEV_ADDR, CMD_STANDBY) #スタンバイモード
  end

  # def conv(ary, base)
  #   ((ary.getbyte(base) << 26) >> 18) + ary.getbyte(base+1)
  # end
  def conv(ary, base)
    ((ary.getbyte(base) <<24) | (ary.getbyte(base + 1) << 16)) >> 18
  end

#   def conv(str, base)
#   lo = str.getbyte(base)
#   hi = str.getbyte(base + 1)
#   v = (hi << 8) | lo
#   v -= 65536 if v >= 32768
#   v
#   end

  def read()
    # @i2c.write(DEV_ADDR, [0x00])
    # val = @i2c.read(DEV_ADDR, 1)
    # p val
    # @i2c.write(DEV_ADDR, [0x00])
    # id = @i2c.read(DEV_ADDR, 1)
    # puts "0xAD?"
    # p id  # ADXL367なら 0xAD が返るはず

    # POWER_CTL = 0x03 にして測定モードへ（bit0=1）
    # @i2c.write(DEV_ADDR, [0x2D, 0x03])
    # # POWER_CTL (0x2D) を読む
    # @i2c.write(DEV_ADDR, [0x2D])
    # pctl = @i2c.read(DEV_ADDR,1)
    
    # puts "POWER_CTL:"
    # p pctl  # measure入れてるなら bit が立つ→ 0x03 になった

    @i2c.write(DEV_ADDR, [REG_DATA])
    #Copro.delayMs(1000)
    val = @i2c.read(DEV_ADDR, 6)
    return nil if val.nil? || val.size < 6
    ADXLResult.new(conv(val, 0), conv(val, 2), conv(val, 4))
  end
end

# === Parameters ===
MIN_SENSITIVITY = 2000
STEP_TIMEOUT = 50
REGULATION_STEPS = 4 #一回１にして実験

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
i2c = I2C.new()
acc = ADXL.new(i2c)
acc.on()
# === Main Loop (LP Core) ===
# Copro.sleep_and_run do
for num in 1..200 do
  v = acc.read()
  if v
    puts "Got Accel Data: x=#{v.x} y=#{v.y} z=#{v.z}"
    # Magnitude
    val_x = v.x.abs
    val_y = v.y.abs
    val_z = v.z.abs
    mag = val_x + val_y + val_z
    
    if ema_mag == 0
      ema_mag = mag
    else
      ema_mag = (ema_mag * 3 + mag) / 4
    end
    
    filtered = ema_mag
    samples_since_change += 1
    # puts "samples_since_change: #{samples_since_change}"
    # puts "ema_mag: #{ema_mag}, max_peak: #{max_peak}"
    if looking_for_max
      if filtered > max_peak
        # puts "a"
        max_peak = filtered
        samples_since_change = 0
      elsif samples_since_change > 5 && (max_peak - filtered) > 500
        # puts "b"
        # Max Peak Detected
        looking_for_max = false
        min_peak = filtered
        samples_since_change = 0
      end
    else
      if filtered < min_peak
        # puts "c"
        min_peak = filtered
        samples_since_change = 0
      elsif samples_since_change > 5 && (filtered - min_peak) > 500
        # puts "d"
        
        # Min Peak (Valley) Detected -> Step Cycle Complete
        peak_diff = max_peak - min_peak
        mid_point = (max_peak + min_peak) / 2
        # puts "cycle done: diff=#{peak_diff} reg=#{reg_mode} steps=#{step_count}"
        puts "CYCLE d: diff=#{peak_diff} max=#{max_peak} min=#{min_peak} filt=#{filtered}"

        # Update Dynamic Threshold
        if dynamic_thresh == 0
          # puts "e"
          dynamic_thresh = mid_point
        else
          # puts "f"
          dynamic_thresh = (dynamic_thresh * 3 + mid_point) / 4
        end
        
        # Validation
        # p "diff: #{peak_diff} min: #{MIN_SENSITIVITY}"
        if peak_diff > MIN_SENSITIVITY
          if reg_mode
            # puts "g"
            step_count += 1
            gpio_state = !gpio_state
            Copro.gpio(1, gpio_state)
            puts "Step! Total: #{step_count}"
          else
            # puts "h"
            consec_steps += 1
            if consec_steps >= REGULATION_STEPS
              # puts "i"
              reg_mode = true
              step_count += consec_steps
              consec_steps = 0
              # puts "Regulation Mode ON! Steps: #{step_count}"
            end
          end
        else
          # Noise
          # puts "Noise ignore: #{peak_diff}"
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
    # puts "mag: #{mag} ema: #{ema_mag} filt: #{filtered} max: #{max_peak} min: #{min_peak} thresh: #{dynamic_thresh} steps: #{step_count}"
  end
  Copro.delayMs(20) # 50Hz
  # puts "end-loop"
end
# end


