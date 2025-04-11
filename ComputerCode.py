# The following code is from 2axisTracker.ipynd:

import serial
import serial.tools.list_ports
import time

def find_pico_port():
    """List all available ports and try to identify the Pico"""
    ports = list(serial.tools.list_ports.comports())
    print("Available ports:")
    for port in ports:
        print(f"Port: {port.device}, Description: {port.description}")
        if "USB Serial Device" in port.description or "Pico" in port.description:
            return port.device
    return None

def connect_to_pico(port=None):
    """
    Connect to Raspberry Pi Pico W via USB serial.
    If port is None, it will try to find the Pico port automatically.
    """
    if port is None:
        port = find_pico_port()
        if port is None:
            raise Exception("Could not find Pico W. Available ports printed above.")
    
    try:
        # Initialize UART with modified settings
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        # Add a longer delay after opening the port
        time.sleep(3)  # Increased to 3 seconds to allow for complete Pico initialization
        
        # Clear startup messages more thoroughly
        while ser.in_waiting:
            startup_msg = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Clearing startup message: {startup_msg}")
            time.sleep(0.1)
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"Successfully connected to {port} using UART")
        return ser
    except (OSError, serial.SerialException) as e:
        print(f"Error connecting to {port}: {e}")
        raise

def send_angles_with_confirmation(ser, pitch, yaw):
    """Send angles to Pico using UART protocol and wait for confirmation"""
    max_retries = 3
    command = f"<{pitch},{yaw}>\n"
    
    for attempt in range(max_retries):
        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send command with UART protocol
        print(f"Attempt {attempt + 1}: Sending via UART: {command.strip()}")
        ser.write(command.encode('utf-8'))
        ser.flush()  # Ensure all data is sent
        
        # Wait for acknowledgment with timeout
        start_time = time.time()
        response_buffer = ""
        while time.time() - start_time < 3.0:  # Increased timeout to 3 seconds
            if ser.in_waiting:
                char = ser.read().decode('utf-8', errors='ignore')
                if char:  # Only process non-empty characters
                    response_buffer += char
                    print(f"Received char: {char!r}")  # Debug each character
                    if char == '\n':
                        print(f"Complete response: {response_buffer.strip()}")
                        if "<OK:" in response_buffer and ">" in response_buffer:
                            print("Position confirmed by Pico via UART")
                            return True
                        response_buffer = ""
            time.sleep(0.01)
        
        print(f"Retry {attempt + 1}/{max_retries}: No valid UART confirmation received")
        time.sleep(1.5)  # Increased delay between retries
    
    print("Failed to get UART confirmation from Pico")
    return False

def servo_sweep():
    try:
        # Connect to Pico W using UART
        ser = connect_to_pico()
        print("Starting servo sweep test using UART protocol...")
        
        # Sweep pattern
        # Format: (pitch, yaw, delay_seconds)
        sweep_pattern = [
            # Center both servos
            (90, 90, 2),
            
            # Pitch sweep (yaw centered)
            (0, 90, 1),    # Up
            (180, 90, 1),  # Down
            (90, 90, 1),   # Center
            
            # Yaw sweep (pitch centered)
            (90, 0, 1),    # Left
            (90, 180, 1),  # Right
            (90, 90, 1),   # Center
            
            # Diagonal sweep
            (45, 45, 1),      # Up-Left
            (135, 135, 1),    # Down-Right
            (45, 135, 1),     # Up-Right
            (135, 45, 1),     # Down-Left
            (90, 90, 1),      # Center
            
            # Circle pattern
            (45, 90, 0.5),    # Top
            (90, 135, 0.5),   # Right
            (135, 90, 0.5),   # Bottom
            (90, 45, 0.5),    # Left
            (90, 90, 1),      # Center
        ]
        
        # Execute sweep pattern with UART communication
        for pitch, yaw, delay in sweep_pattern:
            if not send_angles_with_confirmation(ser, pitch, yaw):
                print("Warning: Position not confirmed over UART, continuing anyway")
            time.sleep(delay)
            
        print("UART sweep test completed!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
            print("UART connection closed")

# Run the sweep test
if __name__ == "__main__":
    servo_sweep()
