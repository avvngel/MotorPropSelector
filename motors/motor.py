#!/usr/bin/env python3

class Motor:
    
    def __init__(
        self,
        name,
        T_stall,
        K_v,
        data_sheet = ""
    ):
        self.name = name
        self.T_stall = T_stall
        self.K_v = K_v
        self.data_sheet = data_sheet

    
