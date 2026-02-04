#!/usr/bin/env python3

import math 

class Prop:

    def __init__(
        self, 
        name,
        Z, # number of blades
        diameter, # meters
        pitch,
        A_E,
        area_ratio_method = 'wageningen'
    ):
        import coeffs as c

        self.name = name
        self.Z = Z
        self.diameter = diameter
        self.pitch = pitch
        self.A_E = A_E
        self.A_O = math.pi/4*diameter**2
    
        if area_ratio_method = 'wageningen':
            self.K_T, self.K_Q = c.wageningen_coeffs(
                0,
                self.pitch,
                self.diameter,
                self.A_E,
                self.A_O,
                self.Z
            )


    
        else:
            print("area_ratio_method " + area_ratio_method + " not implemented.")
            raise NotImplementedError

        

        



