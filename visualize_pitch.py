import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# -------- SERIAL CONFIG --------
PORT = "/dev/cu.usbmodem1101"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# -------- PLOT SETUP --------
plt.ion()
fig, ax = plt.subplots()

ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-0.2, 1.4)
ax.set_aspect("equal")
ax.set_title("Self-Balancing Robot Pitch")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Ground line
ax.plot([-2, 2], [0, 0])

# Robot parameters
robot_length = 1.0   # height of robot
pivot = np.array([0.0, 0.0])  # wheel contact point

# Initial robot line
robot_line, = ax.plot([], [], linewidth=4)

# bot length
length = 1.0

# -------- MAIN LOOP --------
while True:
    try:
        # Flush old samples to reduce lag
        while ser.in_waiting > 1:
            ser.readline()

        line = ser.readline().decode().strip()
        if not line:
            continue

        pitch_deg = float(line)
        pitch_rad = math.radians(pitch_deg)

        x = length * math.sin(pitch_rad)
        y = length * math.cos(pitch_rad)

        robot_line.set_data([0, x], [0, y])

        fig.canvas.draw()
        fig.canvas.flush_events()

    except ValueError:
        pass
    except KeyboardInterrupt:
        break

ser.close()
