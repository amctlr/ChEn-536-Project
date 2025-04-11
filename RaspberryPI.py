# The following code is uploaded to the pico
from machine import Pin, PWM, UART
import time

# At the top
led = Pin("LED", Pin.OUT)  # Built-in LED
# or led = Pin(25, Pin.OUT)  # for external LED

def blink(times):
    for _ in range(times):
        led.on()
        time.sleep_ms(500)  # Increased from 200ms to 500ms
        led.off()
        time.sleep_ms(500)  # Increased from 200ms to 500ms

# Comment out MPU6050 Constants and related code
'''
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

# Initialize I2C
i2c = I2C(1, scl=Pin(27), sda=Pin(26), freq=400000)
'''

# Initialize UART with full configuration
uart = UART(0, 
    baudrate=9600,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(0),  # GP0
    rx=Pin(1)   # GP1
)

# Servo setup
servo1 = PWM(Pin(12))  # Pitch servo
servo2 = PWM(Pin(13))  # Yaw servo
servo1.freq(50)
servo2.freq(50)

def set_servo_angle(servo, angle):
    """Convert angle (0-180) to duty cycle and set servo position"""
    # Convert angle to duty cycle (0.5ms - 2.5ms)
    min_duty = 1638  # 0.5ms (0 degrees)
    max_duty = 8192  # 2.5ms (180 degrees)
    duty = min_duty + (max_duty - min_duty) * angle / 180
    servo.duty_u16(int(duty))

# Initialize servo positions to center and blink 4 times
blink(4)  # Four blinks at initialization
set_servo_angle(servo1, 90)  # Center pitch
set_servo_angle(servo2, 90)  # Center yaw

'''
def init_mpu6050():
    """Initialize the MPU6050 sensor"""
    # Check if MPU6050 is present
    devices = i2c.scan()
    if MPU6050_ADDR not in devices:
        raise RuntimeError("MPU6050 not found on I2C bus")
        
    # Wake up the MPU6050
    try:
        i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, bytes([0]))
        time.sleep_ms(100)
        
        # Configure the sensor
        i2c.writeto_mem(MPU6050_ADDR, SMPLRT_DIV, bytes([7]))
        i2c.writeto_mem(MPU6050_ADDR, GYRO_CONFIG, bytes([0]))
        i2c.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([0]))
    except Exception as e:
        raise RuntimeError(f"Failed to initialize MPU6050: {str(e)}")

def read_raw_data(addr):
    """Read raw 16-bit data from the MPU6050"""
    high = i2c.readfrom_mem(MPU6050_ADDR, addr, 1)[0]
    low = i2c.readfrom_mem(MPU6050_ADDR, addr + 1, 1)[0]
    value = (high << 8) | low
    
    if value > 32767:
        value -= 65536
    return value

def get_data():
    """Get processed data from MPU6050"""
    # Read accelerometer data
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    
    # Read temperature
    temp = read_raw_data(TEMP_OUT_H) / 340.0 + 36.53
    
    # Read gyroscope data
    gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0
    gyro_y = read_raw_data(GYRO_XOUT_H + 2) / 131.0
    gyro_z = read_raw_data(GYRO_XOUT_H + 4) / 131.0
    
    return (accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z)
'''

# main loop:
while True:
    try:
        if uart.any():  # Check if data is available
            led.on()  # Visual indicator of receiving data
            
            # Read the entire line
            input_data = bytearray()
            timeout = time.ticks_ms() + 1000  # 1 second timeout
            
            while time.ticks_ms() < timeout:
                if uart.any():
                    byte = uart.read(1)
                    if byte == b'\n':
                        break
                    input_data.extend(byte)
                time.sleep_ms(10)
            
            input_str = input_data.decode().strip()
            print(f"Received: {input_str}")  # Debug print
            
            if input_str.startswith('<') and input_str.endswith('>'):
                input_str = input_str[1:-1]  # Remove brackets
                try:
                    pitch, yaw = map(int, input_str.split(','))
                    print(f"Parsed angles - Pitch: {pitch}, Yaw: {yaw}")
                    
                    if 0 <= pitch <= 180 and 0 <= yaw <= 180:
                        set_servo_angle(servo1, pitch)
                        set_servo_angle(servo2, yaw)
                        
                        # Send confirmation
                        confirmation = f"<OK:{pitch},{yaw}>\n"
                        uart.write(confirmation.encode())
                        print(f"Sent: {confirmation}")  # Debug print
                        blink(2)  # Success indicator
                    else:
                        uart.write("<ERROR:Invalid angles>\n".encode())
                        blink(3)  # Error indicator
                except:
                    uart.write("<ERROR:Invalid format>\n".encode())
                    blink(3)  # Error indicator
            else:
                uart.write("<ERROR:Invalid format>\n".encode())
                blink(3)  # Error indicator
            
            led.off()
        time.sleep_ms(10)
            
    except Exception as e:
        print(f"Error: {str(e)}")
        uart.write(f"<ERROR:{str(e)}>\n".encode())
        blink(5)  # Error indicator
        time.sleep(1)
