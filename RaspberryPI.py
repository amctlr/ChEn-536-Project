# The following code is uploaded to the pico
from machine import Pin, PWM, UART
import time

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

# Setup UART for serial communication
uart = UART(0, baudrate=9600)

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

# Initialize servo positions to center
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
        # Wait for serial input
        print("Waiting for input...")
        while not uart.any():
            time.sleep_ms(10)
            
        input_str = uart.readline().decode().strip()
        # Remove angle brackets
        if input_str.startswith('<') and input_str.endswith('>'):
            input_str = input_str[1:-1]  # Remove first and last characters
            pitch, yaw = map(int, input_str.split(','))
            
            # Validate angles
            if 0 <= pitch <= 180 and 0 <= yaw <= 180:
                set_servo_angle(servo1, pitch)  # Set pitch
                set_servo_angle(servo2, yaw)    # Set yaw
                
                # Send confirmation in the expected format
                uart.write(f"<OK:{pitch},{yaw}>\n".encode())
            else:
                uart.write("<ERROR:Invalid angles>\n".encode())
        else:
            uart.write("<ERROR:Invalid format>\n".encode())
            
    except ValueError:
        uart.write("<ERROR:Invalid input format>\n".encode())
    except Exception as e:
        uart.write(f"<ERROR:{str(e)}>\n".encode())
        time.sleep(1)


