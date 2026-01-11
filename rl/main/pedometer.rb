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
  CMD_MEASURE = [CTRL_REG, 2]
  CMD_STANDBY = [CTRL_REG, 0]
  CMD_SOFTRESET = [0x1f, 0x52]
  CMD_FIFO = [0x18]
  def initialize(i2c)
    @i2c = i2c
    @i2c.write(DEV_ADDR, CMD_SOFTRESET)
    @i2c.write(DEV_ADDR, [0x29, 1])
    @i2c.write(DEV_ADDR, [0x28, 2])
  end
  def on()
    @i2c.write(DEV_ADDR, CMD_MEASURE)
  end
  def off()
    @i2c.write(DEV_ADDR, CMD_STANDBY)
  end
  def conv(ary, base)
    ((ary.getbyte(base) << 26) >> 18) + ary.getbyte(base+1)
  end
  def read()
    @i2c.write(DEV_ADDR, CMD_FIFO)
    Copro.delayMs(5)
    val = @i2c.read(DEV_ADDR, 6)
    if val.size == 0 then
      return nil
    end
    ADXLResult.new(conv(val, 0), conv(val, 2), conv(val, 4))
  end
end

i2c = I2C.new()
acc = ADXL.new(i2c)
buf = Array.new(12)
THRESHOLD = 2000
GRAVITY = 4000 # 1G / 0.25MG/LSB = 4000
STEPS = 0
LAST_MAG = 0

def read_for_1sec(acc, buffer)
  i = 0
  res = true
  acc.on()
  Copro.delayMs(20)
  while i < 12 do
    Copro.delayMs(80)
    v = acc.read()
    if v.nil? then
      res = false
    end
    buffer[i] = v
    i += 1
  end
  acc.off()
  res
end

Copro.sleep_and_run do
  while true do
    break unless read_for_1sec(acc, buf)
    
    # Process 1 second worth of data
    x = 0; y = 0; z = 0; i = 0
    while i < 12 do
      v = buf[i]
      if v
         # Simple step detection logic
         # Calculate magnitude squared (to avoid sqrt cost if possible, but here we need magnitude for threshold)
         # Using simple Manhattan distance for speed or proper magnitude? 
         # The original C code calculated average magnitude of the *vector sum* of 12 samples.
         # That logic was: sum(x), sum(y), sum(z) -> avg_x, avg_y, avg_z -> abs(avg_vector - gravity) > threshold.
         # That logic checks *orientation change* or *sustained acceleration*?
         # Actually the original code: 
         # vx = (abs(vx) + abs(vy) + abs(vz))/12;
         # if(abs(vx - GRAVITY) > THRESHOLD) goto FAIL;
         # This looks like it's checking if the device is *stable* (Gravity only).
         # BUT for a pedometer, we want to Count Steps.
         # User asked to "Measure power consumption difference of a Pedometer".
         # So I should implement a SIMPLE Step Counter.
         
         # Let's implement a simple Peak Detection:
         # Mag = abs(x)+abs(y)+abs(z)
         # If Mag > Threshold and Last_Mag <= Threshold, Step++
         
         mag = v.x.abs + v.y.abs + v.z.abs
         if mag > (GRAVITY + THRESHOLD)
            if LAST_MAG <= (GRAVITY + THRESHOLD)
               STEPS = STEPS + 1
               # Copro.led_on() if we had one
            end
         end
         LAST_MAG = mag
      end
      i += 1
    end
    
    Copro.delayMs(100) # Wait a bit before next batch? Or just loop?
    # Original 'gps_acc' slept for 7 seconds. 
    # For a Pedometer, we usually sleep less or allow the sensor to FIFO.
    # The requirement is "Pedometer", so continuous monitoring is implied, OR checking FIFO periodically.
    # I will modify the sleep time to be smaller 0 or just loop for continuous measurement.
    # However, to measure power distinctively, maybe we want duties?
    # Let's assume Continuous 10Hz sampling. The `read_for_1sec` already takes 1 second.
    # So we loop immediately.
  end
end
