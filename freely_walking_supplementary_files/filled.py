import h5py
import os
import math

#matplotlib inline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import seaborn as sea
import scipy.signal
import read_data as rd
import organize_data as od
import fill_nans as fill
import velocity as vel 
import swing_stance as s_s 

from scipy import signal
import fill_nans as fill 

def filled(raw_positions):
    
    filled_tx = fill.fill_nans(raw_positions[0])
    filled_ty = fill.fill_nans(raw_positions[1])
    filled_hx = fill.fill_nans(raw_positions[2])
    filled_hy = fill.fill_nans(raw_positions[3])
    filled_ax = fill.fill_nans(raw_positions[4])
    filled_ay = fill.fill_nans(raw_positions[5])
    filled_r1_x = fill.fill_nans(raw_positions[6])
    filled_r1_y = fill.fill_nans(raw_positions[7])
    filled_r2_x = fill.fill_nans(raw_positions[8])
    filled_r2_y = fill.fill_nans(raw_positions[9])
    filled_r3_x = fill.fill_nans(raw_positions[10])
    filled_r3_y = fill.fill_nans(raw_positions[11])
    filled_l1_x = fill.fill_nans(raw_positions[12])
    filled_l1_y = fill.fill_nans(raw_positions[13])
    filled_l2_x = fill.fill_nans(raw_positions[14])
    filled_l2_y = fill.fill_nans(raw_positions[15])
    filled_l3_x = fill.fill_nans(raw_positions[16])
    filled_l3_y = fill.fill_nans(raw_positions[17])

    x_pos=[filled_r1_x, filled_r2_x, filled_r3_x, filled_l1_x, filled_l2_x, filled_l3_x]
    y_pos=[filled_r1_y, filled_r2_y, filled_r3_y, filled_l1_y, filled_l2_y, filled_l3_y]
    body_pos = [filled_hx, filled_hy, filled_tx, filled_ty, filled_ax, filled_ay] 
    
    x_pos_ego =[filled_r1_x-filled_tx, filled_r2_x-filled_tx, filled_r3_x-filled_tx, filled_l1_x-filled_tx, filled_l2_x-filled_tx, filled_l3_x-filled_tx]
    y_pos_ego =[filled_r1_y-filled_ty, filled_r2_y-filled_ty, filled_r3_y-filled_ty, filled_l1_y-filled_ty, filled_l2_y-filled_ty, filled_l3_y-filled_ty]
    body_pos_ego = [filled_hx-filled_tx, filled_hy-filled_ty, filled_tx-filled_tx, filled_ty-filled_ty, filled_ax-filled_tx, filled_ay-filled_ty] 

    return x_pos, y_pos, body_pos, x_pos_ego, y_pos_ego, body_pos_ego
    
