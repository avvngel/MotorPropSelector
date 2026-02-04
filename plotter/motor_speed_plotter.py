#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

def plot_torque_speed_curve(
        motor, 
        operating_voltage, 
        ax = None
):

    no_load_speed = motor.K_v*operating_voltage
    x = np.linspace(0, no_load_speed, 200)
    y = motor.T_stall*(1 - x/no_load_speed)

    if ax is None:
        fig, ax = plt.subplots()
        ax.set(
            xlabel = "Rotations per second",
            ylabel = "Torque (Nm)",
            title = "Motor Speed Curve"
        )

    ax.plot(x, y, label=motor.name)
    ax.legend()
    return ax.figure
    
    

