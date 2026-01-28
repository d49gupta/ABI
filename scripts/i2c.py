import smbus2
import time

# I2C address and Register Pointers
DEVICE_ADDRESS = 0x48
CONFIG_REG_ADDR = 0x01
CONV_REG_ADDR = 0x00

# Your Configuration: AIN0/GND, 6.144V, Continuous Mode
high_byte = 0xC0 
low_byte = 0x83

bus = smbus2.SMBus(1)

bus.write_i2c_block_data(DEVICE_ADDRESS, CONFIG_REG_ADDR, [high_byte, low_byte])
print(f"Config Register set to: {hex(high_byte)} {hex(low_byte)}")
bus.write_byte(DEVICE_ADDRESS, CONV_REG_ADDR)

print("Starting readings... Press Ctrl+C to stop.")

try:
    while True:
        data = bus.read_i2c_block_data(DEVICE_ADDRESS, CONV_REG_ADDR, 2)
        
        # Step 4: Convert bytes to 16-bit signed integer
        # data[0] is MSB, data[1] is LSB
        raw_value = (data[0] << 8) | data[1]
        
        # Handle Two's Complement for negative values
        if raw_value > 32767:
            raw_value -= 65536
            
        voltage = (raw_value * 6.144) / 32768.0
        print(f"Raw: {raw_value:6} | Voltage: {voltage:.4f}V")        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped by user.")
