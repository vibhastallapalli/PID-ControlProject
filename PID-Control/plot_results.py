import csv
import matplotlib.pyplot as plt

# Read data
times = []
setpoints = []
velocities = []
throttles = []
disturbances = []
energies = []
efficiencies = []

try:
    with open('results.csv', 'r') as f:
        reader = csv.reader(f)
        next(reader) # Skip header
        for row in reader:
            if not row: continue
            times.append(float(row[0]))
            setpoints.append(float(row[1]))
            velocities.append(float(row[2]))
            throttles.append(float(row[3]))
            disturbances.append(float(row[4]))
            energies.append(float(row[5]))
            efficiencies.append(float(row[6]))
except FileNotFoundError:
    print("Error: results.csv not found")
    exit(1)

# Create plots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=False)

# Plot 1: Velocity vs Setpoint
ax1.plot(times, velocities, label='Velocity (m/s)', color='b')
ax1.plot(times, setpoints, label='Setpoint (m/s)', color='g', linestyle='--')
ax1.set_ylabel('Speed (m/s)')
ax1.set_title('E-bike Velocity Controller Response')
ax1.legend()
ax1.grid(True)

# Plot 2: Throttle & Disturbance
ax2.plot(times, throttles, label='Throttle (0-1)', color='r')
ax2.set_ylabel('Throttle')
ax2.set_xlabel('Time (s)')
ax2.set_ylim(-0.1, 1.1)

# Create a twin axis for disturbance
ax2_right = ax2.twinx()
ax2_right.plot(times, disturbances, label='Disturbance (N)', color='purple', linestyle=':')
ax2_right.set_ylabel('Disturbance Force (N)', color='purple')
ax2_right.tick_params(axis='y', labelcolor='purple')

ax2.legend(loc='upper left')
ax2_right.legend(loc='upper right')
ax2.grid(True)

# Plot 3: Efficiency vs Velocity (Scatter or Line?)
# User asked for "Energy Efficiency vs. Velocity". 
# Since we have time series, we can plot Efficiency vs Velocity.
# Note: Efficiency is noisy if power is near zero or fluctuating. 
# We'll plot it as a scatter to see the trend.
ax3.scatter(velocities, efficiencies, label='Instantaneous Efficiency (m/J)', color='orange', s=10, alpha=0.5)
ax3.set_xlabel('Velocity (m/s)')
ax3.set_ylabel('Efficiency (m/J)')
ax3.set_title('Energy Efficiency vs. Velocity')
ax3.grid(True)
# Limit y-axis if efficiency spikes to infinity
ax3.set_ylim(0, 0.05) # Expected range? m/J. 10m/s / 100N*10m/s = 10/1000 = 0.01 m/J.
                      # Approx 0.005 - 0.02 is likely range. 

plt.tight_layout()
plt.savefig('simulation_results.png')
print("Plot saved to simulation_results.png")
