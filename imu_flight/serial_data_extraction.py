## extracting data from arduino 
## as there is no certain library for ros2 
## in arduino env 

import serial
import time

# Open the serial port where the Arduino is connected
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Change 'COM3' to your specific port
time.sleep(2)  # Wait for the connection to stabilize

ax = 0
ay = 0
az = 0
roll = 0
pitch = 0
yaw = 0

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()  # Read and decode the serial data
        print("Raw data:", line)  # Print raw data for debugging

        # Parse the data into individual variables
        try:
            data = line.split(',')  # Split by commas since the Arduino sends CSV data
            
            # Ensure the correct number of elements are parsed
            if len(data) == 6:
                ax = float(data[0])
                ay = float(data[1])
                az = float(data[2])
                roll = float(data[3])
                pitch = float(data[4])
                yaw = float(data[5])

                # Print individual values
                print(f"Acceleration X: {ax}, Y: {ay}, Z: {az}")
                print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
            else:
                print("Error: Incomplete data received.")
                
        except ValueError as e:
            print(f"Error parsing data: {e}, skipping this line.")
