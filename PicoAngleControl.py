## this code actually works

from machine import Pin, PWM
import time

# Setup LED and servo pins
led = Pin(25, Pin.OUT)
pitch_servo = PWM(Pin(12))
yaw_servo = PWM(Pin(13))

# Configure PWM for servos
pitch_servo.freq(50)  # 50Hz frequency for servos
yaw_servo.freq(50)
print("LED and Servo Control Ready")

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
                
                # Convert pitch angle to duty cycle
                pitch_duty = int(1000 + ((pitch_angle + 90)/180) * 8000)
                pitch_servo.duty_u16(pitch_duty)
                
                # Convert yaw angle to duty cycle  
                yaw_duty = int(1000 + ((yaw_angle + 90)/180) * 8000)
                yaw_servo.duty_u16(yaw_duty)
                
                print(f"Angles set - Pitch: {pitch_angle}, Yaw: {yaw_angle}")
            except:
                print("Invalid angle command. Format: ANGLE pitch_angle yaw_angle")
        elif data == "ON":
            led.value(1)
        elif data == "OFF":
            led.value(0)
            
        time.sleep(0.1)  # Small delay to prevent double-processing
    except Exception as e:
        print(f"Error occurred: {e}")
