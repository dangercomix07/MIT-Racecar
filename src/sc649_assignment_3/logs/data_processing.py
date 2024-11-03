import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load data from CSV
data = pd.read_csv('data_log.csv')

time = data['Time'].values
X = data['PositionX'].values
Y = data['PositionY'].values
Phi = data['SteeringAngle'].values
XError = data['XError'].values
YError = data['YError'].values
HeadingError = data['HeadingError'].values

tracking_error = np.sqrt((XError)**2 + (YError)**2)

plt.figure()
plt.plot(time, tracking_error, label="Tracking Error")
plt.xlabel("Time (s)")
plt.ylabel("Error (m)")
plt.title("Tracking Error Over Time")
plt.legend()
plt.show()

# Plot trajectory
plt.plot(X, Y, label='Actual Trajectory')
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectory Tracking")
plt.legend()
plt.show()
