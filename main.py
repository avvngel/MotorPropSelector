#!/usr/bin/env python3

from motors.motor import *
from plotter.motor_speed_plotter import * 
import motors.motors as motors
import matplotlib.pyplot as plt

if __name__ == "__main__":

    figno = plot_torque_speed_curve(motors.D65L120_50, 24)
    plt.show()
    

