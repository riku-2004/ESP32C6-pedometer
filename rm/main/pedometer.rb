#
# Pedometer Algorithm (Unified with cl/cm)
# Lightweight Dynamic Threshold using EMA
#

class I2C
  def initialize()
    puts "[rb] I2C.new start"
    Copro.i2cinit()
    puts "[rb] I2C.new done"
  end
  def read(addr, len, reg = nil)
    if reg
      Copro.i2cwrite(addr, [reg])
      Copro.delayMs(1)
    end
    Copro.i2cread(addr, len)
  end
  def write(addr, data)
    Copro.i2cwrite(addr, data)
  end
end

class ADXL367
  attr_reader :x, :y, :z

  def initialize(i2c)
    @i2c = i2c
    init_sensor
  end

  def init_sensor
    # Soft Reset
    @i2c.write(0x1D, [0x1F, 0x52])
    Copro.delayMs(10)
    # Filter Control (100Hz)
    @i2c.write(0x1D, [0x2C, 0x13])
    Copro.delayMs(10)
    # Power Control (Measure)
    @i2c.write(0x1D, [0x2D, 0x02])
    Copro.delayMs(50)
    puts "ADXL367 initialized"
  end

  def read
    # Read 6 bytes starting from 0x0E (XDATA_L)
    data = @i2c.read(0x1D, 6, 0x0E)
    if data
      @x = conv(data.getbyte(0), data.getbyte(1))
      @y = conv(data.getbyte(2), data.getbyte(3))
      @z = conv(data.getbyte(4), data.getbyte(5))
      return true
    end
    return false
  end

  def conv(lsb, msb)
    val = (msb << 8) | lsb
    val = val >> 2 # 14bit
    if (val & 0x2000) != 0
      val = val - 16384 # 2's complement
    end
    val
  end
end

class Pedometer
  attr_reader :step_count

  # Parameters
  MIN_SENSITIVITY = 2000
  STEP_TIMEOUT = 50
  REGULATION_STEPS = 4

  def initialize
    @step_count = 0
    @ema_mag = 0
    @dynamic_thresh = 0
    @max_peak = 0
    @min_peak = 0
    @looking_for_max = true
    @samples_since_change = 0
    @reg_mode = false
    @consec_steps = 0
  end

  def process(x, y, z)
    # Magnitude (Manhattan)
    mag = x.abs + y.abs + z.abs
    
    # EMA LPF
    if @ema_mag == 0
      @ema_mag = mag
    else
      @ema_mag = (@ema_mag * 3 + mag) / 4
    end
    
    filtered = @ema_mag
    @samples_since_change += 1

    if @looking_for_max
      if filtered > @max_peak
        @max_peak = filtered
        @samples_since_change = 0
      elsif @samples_since_change > 5 && (@max_peak - filtered) > 500
        # Peak detected
        @looking_for_max = false
        @min_peak = filtered
        @samples_since_change = 0
      end
    else
      if filtered < @min_peak
        @min_peak = filtered
        @samples_since_change = 0
      elsif @samples_since_change > 5 && (filtered - @min_peak) > 500
        # Valley detected (One step cycle complete)
        peak_diff = @max_peak - @min_peak
        mid_point = (@max_peak + @min_peak) / 2
        
        # Update dynamic threshold (EMA)
        if @dynamic_thresh == 0
          @dynamic_thresh = mid_point
        else
          @dynamic_thresh = (@dynamic_thresh * 3 + mid_point) / 4
        end
        
        # Step validation
        if peak_diff > MIN_SENSITIVITY
          if @reg_mode
            @step_count += 1
            puts "Step! Total: #{@step_count}"
          else
            @consec_steps += 1
            if @consec_steps >= REGULATION_STEPS
              @reg_mode = true
              @step_count += @consec_steps
              @consec_steps = 0
              puts "Regulation Mode ON! Steps: #{@step_count}"
            end
          end
        else
          # Noise
          unless @reg_mode
            @consec_steps = 0
          end
        end
        
        @looking_for_max = true
        @max_peak = filtered
        @samples_since_change = 0
      end
    end
    
    # Timeout
    if @samples_since_change > STEP_TIMEOUT
      @looking_for_max = true
      @max_peak = filtered
      unless @reg_mode
        @consec_steps = 0
      end
    end
    
    return mag
  end
end

# Main Loop
i2c = I2C.new
adxl = ADXL367.new(i2c)
ped = Pedometer.new
debug_cnt = 0

puts "Starting Pedometer (Ruby/rm)"

while true
  if adxl.read
    mag = ped.process(adxl.x, adxl.y, adxl.z)
    debug_cnt += 1
    if debug_cnt >= 50  # 1秒ごと (50Hz)
      puts "Mag: #{mag}, Steps: #{ped.step_count}"
      debug_cnt = 0
    end
  end
  Copro.delayMs(20) # 50Hz
end

