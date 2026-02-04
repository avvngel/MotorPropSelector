#!/usr/bin/env python3

r_si      = 0.015        # m   (assumed, see decomposition above)
d_sp      = 0.004        # m
d_st      = 0.010        # m
w_st      = 0.006283     # m   (= 0.5 * tau_u)
l_st      = 0.165        # m   (datasheet L165)
Q         = 12           #     (datasheet 12N/10P)
y         = 1            #     (tooth-coil)
z_Q       = 2.5          #     (datasheet "2.5T")
z_C       = 4            #     (4 tooth-coils in series per phase)
Kcu       = 0.45         #     (assumption)
Kov       = 1.0          #     (tooth-coil; unused if y=1)
sigma_cond= 5.77e7       # S/m  (20°C; scale with T if desired)
slot_area = 6.283e-5     # m^2  (~62.83 mm^2)
n_layers  = 1

