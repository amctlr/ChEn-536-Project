# ChEn-536-Project

## From ChatGPT:

Step 1: Install CircuitPython on the Pico W
Download the latest CircuitPython UF2 for the Pico W:
https://circuitpython.org/board/raspberry_pi_pico_w/

Put your Pico W in BOOTSEL mode (hold the BOOTSEL button while plugging in USB).

Drag-and-drop the UF2 file onto the RPI-RP2 drive.

The Pico will reboot and appear as a USB drive called CIRCUITPY.


Step 2: Install adafruit-blinka on Your PC
CircuitPython makes it possible to write Python scripts on your computer and control the Pico remotely. To do this, install Adafruit Blinka:

bash
Copy
Edit
pip install adafruit-blinka
pip install pyserial
Now, check if your computer detects the Pico W:

bash
Copy
Edit
python -m serial.tools.list_ports
You should see something like:

bash
Copy
Edit
/dev/ttyUSB0  (Linux/macOS)
COM3          (Windows)
