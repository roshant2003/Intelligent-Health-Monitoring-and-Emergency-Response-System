from sklearn.externals import joblib
import numpy as np

# Load the saved model file       give the file path appropriately or have it in the same working directory that will do ig
saved_objects = joblib.load('neural_network_model_with_scaler.pkl')

# Extract the model and scaler from the saved dictionary
loaded_model = saved_objects['model']
loaded_scaler = saved_objects['scaler']

# Sample input values
heart_rate_value = 72               #take the input from the arduino ide (the sensor)
diastolic_bp_value = 80             #default value
systolic_bp_value = 120             #default value
temperature_value = 36.5            #take the input from the arduino ide (the sensor)
sp_o2_value = 99                    #take the input from the arduino ide (the sensor)

# Convert input values to a 1D array
input_values = np.array([heart_rate_value, diastolic_bp_value, systolic_bp_value, temperature_value, sp_o2_value])

# Reshape the input array to a 2D array with a single row
input_values_2d = input_values.reshape(1, -1)

# Normalize the input values using the loaded scaler
input_values_normalized = loaded_scaler.transform(input_values_2d)

# Make predictions using the loaded model
predicted_respiratory_rate = loaded_model.predict(input_values_normalized)

# Print the predicted respiratory rate
print("Predicted Respiratory Rate:", float(predicted_respiratory_rate))
