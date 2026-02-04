#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math

RPM_TO_SI_CONV_FACTOR = 2*math.pi/60
KT_TO_Q_BASIS_CONV_FACTOR = 1/np.sqrt(2)

K_v_ll = 120*RPM_TO_SI_CONV_FACTOR
K_b_ll = 1/K_v_ll



K_t_q = K_b_ll*KT_TO_Q_BASIS_CONV_FACTOR
i_peak = 170*np.sqrt(3/2)
i_nl = 1.8*np.sqrt(3/2)
R_eq = .030/2
V_bus = 60/np.sqrt(3)
u = 1
d = 1
V_app = d*u*V_bus
T_max = 13.7
W_max = 12400
omega_nl = V_app/K_t_q
b = np.sqrt(1/3)*8.4*i_nl/omega_nl

def get_T(rpm):
    omega = rpm*2*math.pi/60
    return K_t_q/R_eq*(V_app - K_t_q*omega) - b


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

    omega_m = np.linspace(1, omega_nl, 1000)
    T = K_t_q/R_eq*(V_app - K_t_q*omega_m) - b
    x = omega_m/(2*math.pi)*60
    for i in range(len(T)):
        w = omega_m[i]
        T[i] = min([T[i], T_max, W_max/w,
                   (-w + np.sqrt(w**2 + 4*R_eq/K_t_q**2*W_max))/(2*R_eq/K_t_q**2)])
    W_mech = T*omega_m
    #W_el = T/K_t*d*V_bus
    plt.plot(x, T)
    plt.plot(x, W_mech/1000)
    #plt.plot(x, W_el/1000)
    plt.title("Torque-Speed Curve")
    plt.show()


