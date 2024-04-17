import serial

# Define the serial port and baud rate
serial_port = 'COM6'  # Change this to the appropriate port
baud_rate = 115200  # Match this with the baud rate set in your Arduino code

# Open serial connection
ser = serial.Serial(serial_port, baud_rate)

try:
    # Loop to continuously read serial data
    while True:
        # Read a line of data from the serial port
        line = ser.readline().decode('utf-8').strip()
        
        # Print the received data
        print(line)

# Handle keyboard interrupt (Ctrl+C)
except KeyboardInterrupt:
    print("Keyboard Interrupt detected. Exiting...")
    # Close the serial connection
    ser.close()
