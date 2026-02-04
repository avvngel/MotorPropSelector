#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math

K_v = 120#50#78
K_t = 60/(2*math.pi*K_v)
i_peak = 170#50#120
i_nl = 1.8#1.2#1.5
R_eq = .095
V_bus = 60#100#159.6
u = 1#.92#.457#.3
d = 1#.5#24/V_bus
V_app = d*u*V_bus
T_max = 13.7#9.5#13.9
W_max = 12400#3500#12800
omega_nl = V_app/K_t
b = 2#8.4*i_nl/omega_nl

def get_T(rpm):
    omega = rpm*2*math.pi/60
    return K_t/R_eq*(V_app - K_t*omega) - b


if __name__ == "__main__":
    
    #print(T_max/(K_t*i_peak))
    #print((K_t*i_peak) - T_max)
    print(b)
    print(get_T(6309) - 3.968)
    print(get_T(6143) - 5.048)
    print(get_T(5971) - 6.224)
    print(get_T(5838) - 7.168)
    print(get_T(5701) - 8.184)
    print(get_T(5565) - 9.176)
    print(get_T(5444) - 10.04)
    print(get_T(5900))


    omega_m = np.linspace(1, omega_nl, 1000)
    T = K_t/R_eq*(V_app - K_t*omega_m) - b
    x = omega_m/(2*math.pi)*60
    for i in range(len(T)):
        w = omega_m[i]
        T[i] = min([T[i], T_max, W_max/w,
                   (-w + np.sqrt(w**2 + 4*R_eq/K_t**2*W_max))/(2*R_eq/K_t**2)])
    W_mech = T*omega_m
    #W_el = T/K_t*d*V_bus
    plt.plot(x, T)
    plt.plot(x, W_mech/1000)
    plt.plot([5444, 5565, 5701, 5838, 5971, 6143, 6309],
            list(reversed([3.968, 5.048, 6.224, 7.168, 8.184, 9.176, 10.04])))
    #plt.plot(x, W_el/1000)
    plt.title("Torque-Speed Curve")
    plt.show()


