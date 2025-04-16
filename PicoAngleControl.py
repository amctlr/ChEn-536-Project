from machine import Pin, PWM, I2C
import time
import math

# Setup LED and servo pins
led = Pin(25, Pin.OUT)
pitch_servo = PWM(Pin(10))
yaw_servo = PWM(Pin(12))

# Configure PWM for servos
pitch_servo.freq(50)  # 50Hz frequency for servos
yaw_servo.freq(50)

# Define PWM ranges based on calibration
YAW_LOWER_PWM = 1960
YAW_HIGHER_PWM = 8260
PITCH_LOWER_PWM = 1940
PITCH_HIGHER_PWM = 8260

# MPU6050 Constants
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize I2C for MPU6050
i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)

# Global variables for yaw tracking
last_time = time.ticks_ms()
current_yaw = 90  # Start at center position
tracking_yaw = False  # Flag to control yaw tracking
tracking_start_time = 0  # Track when yaw tracking started

def init_mpu6050():
    """Initialize the MPU6050 sensor"""
    i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, bytes([0]))
    i2c.writeto_mem(MPU6050_ADDR, SMPLRT_DIV, bytes([7]))
    i2c.writeto_mem(MPU6050_ADDR, GYRO_CONFIG, bytes([0]))  # ±250 deg/s
    i2c.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([0]))

def read_raw_data(addr):
    """Read raw 16-bit data from the MPU6050"""
    high = i2c.readfrom_mem(MPU6050_ADDR, addr, 1)[0]
    low = i2c.readfrom_mem(MPU6050_ADDR, addr + 1, 1)[0]
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_angles():
    """Get pitch and yaw angles from MPU6050 (0 to 180 degrees)"""
    global last_time, current_yaw, tracking_yaw, tracking_start_time
    
    # Read accelerometer data for pitch
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    
    # Calculate pitch from accelerometer data (-90 to 90 degrees)
    pitch = -math.degrees(math.atan2(accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z)))
    if tracking_yaw:
        current_time = time.ticks_ms()
        # Check if 0.5 seconds have elapsed since tracking started
        if time.ticks_diff(current_time, tracking_start_time) > 500:
            tracking_yaw = False
        else:
            # Read gyroscope data for x and y axes
            gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0  # ±250 deg/s sensitivity
            gyro_y = read_raw_data(GYRO_XOUT_H + 2) / 131.0  # ±250 deg/s sensitivity
            
            # Calculate time difference
            dt = time.ticks_diff(current_time, last_time) / 1000.0  # Convert to seconds
            last_time = current_time
            
            # Calculate weights based on current pitch (0-180)
            # At pitch=90 (level), x weight is 1 and y weight is 0
            # At pitch=0 or 180 (vertical), x weight is 0 and y weight is 1
            pitch_rad = math.radians(pitch)
            x_weight = math.cos(pitch_rad)
            y_weight = math.sin(pitch_rad)
            
            # Update yaw angle based on weighted x and y gyro data
            yaw_change = (gyro_x * x_weight + gyro_y * y_weight) * dt
            current_yaw += yaw_change
            
            # Keep yaw in 0-180 range
            current_yaw = current_yaw % 180
    # Convert pitch from -90/90 to 0/180 range
    pitch = pitch + 90
    
    return pitch, current_yaw

# Initialize MPU6050
init_mpu6050()
print("LED, Servo Control and MPU6050 Ready")

while True:
    try:
        data = input().strip()  # Remove any extra whitespace
        if not data:  # Skip empty lines
            continue
        print(f"Debug: Received '{data}'")
        
        # Parse input data
        if data.startswith("ANGLE"):
            try:
                # Split the data into parts and extract angles as floats
                _, pitch_angle, yaw_angle = data.split()
                pitch_angle = float(pitch_angle)
                yaw_angle = float(yaw_angle)
                
                # Convert pitch angle to duty cycle using calibrated values
                pitch_duty = int(PITCH_LOWER_PWM + (pitch_angle/180) * (PITCH_HIGHER_PWM - PITCH_LOWER_PWM))
                pitch_servo.duty_u16(pitch_duty)
                
                # Convert yaw angle to duty cycle using calibrated values
                tracking_yaw = True  # Start tracking yaw when servo moves
                tracking_start_time = time.ticks_ms()  # Record when tracking started
                yaw_duty = int(YAW_LOWER_PWM + (yaw_angle/180) * (YAW_HIGHER_PWM - YAW_LOWER_PWM))
                yaw_servo.duty_u16(yaw_duty)
                
                # Get actual angles from MPU6050
                actual_pitch, actual_yaw = get_angles()
                print(f"MPU: {actual_pitch} {actual_yaw}")
                
            except Exception as e:
                print(f"Invalid angle command. Format: ANGLE pitch_angle yaw_angle. Error: {e}")
        elif data == "ON":
            led.value(1)
        elif data == "OFF":
            led.value(0)
        elif data == "GET_ANGLES":
            actual_pitch, actual_yaw = get_angles()
            print(f"MPU: {actual_pitch} {actual_yaw}")
            
        time.sleep(0.1)  # Small delay to prevent double-processing
    except Exception as e:
        print(f"Error occurred: {e}")
