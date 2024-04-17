import serial
import time
# from sklearn.externals import joblib
import joblib
import numpy as np

# Open serial connection to Arduino
arduino_port = 'COM6'  # Change this to your Arduino port
arduino_baudrate = 115200
ser = serial.Serial(arduino_port, arduino_baudrate, timeout=1)
saved_objects = joblib.load('neural_network_model_with_scaler.pkl')

# Extract the model and scaler from the saved dictionary
loaded_model = saved_objects['model']
loaded_scaler = saved_objects['scaler']

heart_rate_value = 72               #take the input from the arduino ide (the sensor)
diastolic_bp_value = 80             #default value
systolic_bp_value = 120             #default value
temperature_value = 36.5            #take the input from the arduino ide (the sensor)
sp_o2_value = 99   

# Function to send a value to Arduino
def send_value_to_arduino(value):
    ser.write(str(value).encode())
    print(f"Sent value '{value}' to Arduino")
    sensor_data = ser.readline().decode().strip()
    print(sensor_data)

# Read sensor values from Arduino
while True:
    sensor_data = ser.readline().decode('utf-8').strip() 
    # print(sensor_data)
    print(sensor_data.split(";"))
    if sensor_data:
        if sensor_data.startswith("Average"):
            values=sensor_data.split(";")
            sp_o2_value=float(values[0].split(':')[1])
            heart_rate_value=float(values[1].split(':')[1])
            temperature_value =float(values[2].split(':')[1])
            print(sp_o2_value)
            print(heart_rate_value)
            print(temperature_value)
        # ecg_value=float(values.split(':')[7])
        else:
            heart_rate_value = 72 
            temperature_value = 36.5            #take the input from the arduino ide (the sensor)
            sp_o2_value = 99 

        
        input_values = np.array([heart_rate_value, diastolic_bp_value, systolic_bp_value, temperature_value, sp_o2_value])

# Reshape the input array to a 2D array with a single row
        input_values_2d = input_values.reshape(1, -1)

# Normalize the input values using the loaded scaler
        input_values_normalized = loaded_scaler.transform(input_values_2d)

# Make predictions using the loaded model
        predicted_respiratory_rate = loaded_model.predict(input_values_normalized)

# Print the predicted respiratory rate
        print("Predicted Respiratory Rate:", float(predicted_respiratory_rate)) 


        # Example: Send a specific value back to Arduino (e.g., "1" for LED on)
        send_value_to_arduino(float(predicted_respiratory_rate))  # Send "1" to Arduino

    time.sleep(1)  # Adjust sleep time as needed

