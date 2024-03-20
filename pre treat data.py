import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load your data
file_path = r'C:\Users\dell\Downloads\plot_data.csv'  # Adjust the path as needed
data = pd.read_csv(file_path)
measurements = data['value'].values  # Assuming 'value' is the column you want to filter

# Kalman filter parameters
initial_state = measurements[0]
initial_uncertainty = 1.0
process_variance = 1e-7  # Process noise, adjust as needed
measurement_variance = 0.04 ** 2  # Measurement noise, adjust as needed
state_estimate = initial_state
uncertainty_estimate = initial_uncertainty

# Storage for filtered values
filtered_values = []

# Kalman filter algorithm
for measurement in measurements:
    # Prediction step
    state_predict = state_estimate
    uncertainty_predict = uncertainty_estimate + process_variance
    
    # Update step
    kalman_gain = uncertainty_predict / (uncertainty_predict + measurement_variance)
    state_estimate = state_predict + kalman_gain * (measurement - state_predict)
    uncertainty_estimate = (1 - kalman_gain) * uncertainty_predict
    
    filtered_values.append(state_estimate)

# Plotting the filtered values
plt.figure(figsize=(10, 6))
plt.plot(data['elapsed time'], filtered_values, label='Kalman Filtered', linewidth=2)
plt.title('Kalman Filtered Data')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.legend()
plt.show()




# Creating a new DataFrame to store the elapsed time and the filtered values
filtered_data = pd.DataFrame({
    'Elapsed_Time': data['elapsed time'],  # Copying the elapsed time from the original data
    'Filtered_Value': filtered_values     # The filtered values obtained from the Kalman filter
})

# Specify the path where you want to save the new CSV file
output_file_path = r'C:\Users\dell\AppData\Local\Programs\Python\Python312'  # Adjust the path and filename as needed

# Save the DataFrame to a CSV file
filtered_data.to_csv(output_file_path, index=False)

# Print a confirmation message
print(f"Filtered data saved to {output_file_path}")
