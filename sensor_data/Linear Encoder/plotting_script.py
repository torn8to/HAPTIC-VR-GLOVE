from serial import Serial
import math
import numpy as np
import matplotlib.pyplot as plt

# Set up serial connection with Arduino
ser = Serial('COM5', 115200)
ser.flushInput()

buffer_length = 100

# Initialize plot
plt.ion()
fig, ax = plt.subplots()
va_line, = ax.plot([], [])
vb_line, = ax.plot([], [])
atan_line, = ax.plot([], [])

#  Define function to update plot
def update_plot(va,vb,atan):
    a = np.arange(len(va))
    b = np.arange(len(vb))
    tan = np.arange(len(atan))
    va_line.set_data(a, va)
    vb_line.set_data(b, vb)
    atan_line.set_data(tan, atan)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()


va_values = []
vb_values = []
atan_values = []

max_sin = 0
max_cos = 0
min_sin = 4096
min_cos = 4096
sin_scaled = 0
cos_scaled = 0
rotations = 0

def remap(value, old_min, old_max, new_min, new_max):
    if old_max == old_min:
        return new_min
    return (((value - old_min) * (new_max-new_min)) / (old_max - old_min)) + new_min

while True:
    ser_bytes = ser.readline()
    decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8").split(',')
    cos = float(decoded_bytes[1])
    sin = float(decoded_bytes[2])

    if(sin > max_sin):
        max_sin = sin
        
    if(cos > max_cos):
        max_cos = cos

    if(sin < min_sin):
        min_sin = sin

    if(cos < min_cos):
        min_cos = cos

    sin_scaled = remap(sin, min_sin, max_sin, -1, 1)

    cos_scaled = remap(cos, min_cos, max_cos, -1, 1)

    va_values.append(sin_scaled)
    vb_values.append(cos_scaled)
    atan_values.append(math.atan2(sin_scaled, cos_scaled))
    if(len(va_values) > buffer_length):
        va_values.pop(0)
        vb_values.pop(0)
        atan_values.pop(0)
    update_plot(va_values,vb_values,atan_values)

