# for second set of data
import serial
import time
import math
import csv
from gekko import GEKKO
import numpy as np
import matplotlib.pyplot as plt

ser = serial.Serial(
    port='COM6',
    baudrate=115200,
    timeout=1
)

# List to store results for each run
results = {i: [] for i in range(1, 55)}  # Initialize results dict for 54 runs

def send_command(command):
    ser.reset_input_buffer()
    ser.write(f"{command}\r\n".encode())
    time.sleep(0.01)
    
    while ser.in_waiting:
        response = ser.readline().decode().strip()
        if response:
            print(f"Pico response: {response}")
            if response.startswith("MPU:"):
                return [float(x) for x in response[4:].split()]
    return None

def send_angles(pitch, yaw):
    command = f"ANGLE {pitch} {yaw}"
    return send_command(command)

def setup_mpc():
    m = GEKKO()
    m.time = np.linspace(0, 2, 21)
    
    yaw_measured = m.CV(value=0)
    servo_angle = m.MV(value=90)
    
    servo_angle.LOWER = 0
    servo_angle.UPPER = 180
    servo_angle.DMAX = 10
    servo_angle.STATUS = 1
    
    yaw_measured.STATUS = 1
    
    K = 0.8
    tau = 0.5
    m.Equation(tau * yaw_measured.dt() == -yaw_measured + K * servo_angle)
    
    m.options.IMODE = 6
    m.options.CV_TYPE = 2
    m.options.NODES = 3
    
    return m, yaw_measured, servo_angle

def correct_yaw_mpc(target_yaw, actual_yaw, current_servo_yaw):
    if not hasattr(correct_yaw_mpc, 'mpc'):
        correct_yaw_mpc.mpc, correct_yaw_mpc.yaw_cv, correct_yaw_mpc.servo_mv = setup_mpc()
    
    m = correct_yaw_mpc.mpc
    yaw_cv = correct_yaw_mpc.yaw_cv
    servo_mv = correct_yaw_mpc.servo_mv
    
    yaw_cv.MEAS = actual_yaw
    yaw_cv.SP = target_yaw
    
    try:
        m.solve(disp=False)
        new_servo_yaw = float(servo_mv.NEWVAL)
        new_servo_yaw = max(0, min(180, new_servo_yaw))
        return new_servo_yaw
    except:
        print("MPC solution failed, using current angle")
        return current_servo_yaw

try:
    time.sleep(0.01)
    ser.reset_input_buffer()
    
    # Read from angle_tracking_all_hr.csv
    ########
    # INPUT FILE HERE
    ########
    with open('angle_tracking_all_hr.csv', 'r') as file:
        csv_reader = csv.DictReader(file)
        
        # Process each run (1-54)
        for run in range(1, 55):
            # Reset pitch to 25 degrees before each run
            _ = send_angles(25, 90)  # Reset to 25 degrees pitch, neutral yaw
            time.sleep(0.5)  # Give time for servo to reach position
            
            command_count = 0
            pitch_col = f'pitch_{run}'
            yaw_col = f'yaw_{run}'
            
            # Reset file pointer for each run
            file.seek(0)
            next(csv_reader)  # Skip header
            
            
            current_servo_yaw = target_yaw
            actual_angles = send_angles(target_pitch, current_servo_yaw)
            
                
            target_pitch = float(row[pitch_col])
            target_yaw = float(row[yaw_col])
            
            # Ensure angles stay within bounds
            target_pitch = max(0, min(180, target_pitch))
            target_yaw = max(0, min(180, target_yaw))
            
            current_servo_yaw = target_yaw
            actual_angles = send_angles(target_pitch, current_servo_yaw)
            
            if actual_angles:
                actual_pitch, actual_yaw = actual_angles
                pitch_error = abs(target_pitch - actual_pitch)
                yaw_error = abs(target_yaw - actual_yaw)
                
                # Store results for this run
                results[run].append({
                    f'pitch_{run}': target_pitch,
                    f'measured_pitch_{run}': actual_pitch,
                    f'err_{run}': pitch_error,
                    f'yaw_{run}': target_yaw,
                    f'measured_yaw_{run}': actual_yaw,
                    f'yaw_err_{run}': yaw_error
                })
                
                print(f"Run {run} - Command {command_count + 1}")
                print(f"Target angles - Pitch: {target_pitch:.2f}, Yaw: {target_yaw:.2f}")
                print(f"Actual angles - Pitch: {actual_pitch:.2f}, Yaw: {actual_yaw:.2f}")
                print(f"Errors - Pitch: {pitch_error:.2f}, Yaw: {yaw_error:.2f}")
                
                current_servo_yaw = correct_yaw_mpc(target_yaw, actual_yaw, current_servo_yaw)
                print(f"Corrected servo yaw: {current_servo_yaw:.2f}\n")
            
            command_count += 1
            time.sleep(0.01)

except KeyboardInterrupt:
    print("\nOperation interrupted by user")

finally:
    # Save all results to a single CSV
    all_data = []
    max_rows = 0
    
    # First find the maximum number of rows across all runs
    for run in range(1, 55):
        if results[run]:
            max_rows = max(max_rows, len(results[run]))
    
    # Create the combined data
    for i in range(max_rows):
        row_data = {}
        for run in range(1, 55):
            if results[run] and i < len(results[run]):
                row_data.update(results[run][i])
        all_data.append(row_data)
    
    if all_data:
        #########
        # OUTPUT FILE NAME HERE
        ########
        filename = 'combined_results2.csv'  # Changed filename
        fieldnames = []
        for run in range(1, 55):
            if results[run]:
                fieldnames.extend(list(results[run][0].keys()))
        
        with open(filename, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_data)
            print(f"All results saved to {filename}")
    
    ser.close()
    print("Serial connection closed")
