import numpy as np 

def organize_data(fly_dataframes):
    
    # calib = 0.026 - old calibration, updated 03/09/23 
    calib_x = 0.0244 # updated calibration 12/28/23 
    calib_y = 0.0242

    
    frame_idx = fly_dataframes['frame_idx'].values.astype(float)

    # thorax
    tx=fly_dataframes['thorax.x'].values.astype(float)*calib_x
    ty=fly_dataframes['thorax.y'].values.astype(float)*calib_y
    
    # head
    hx=fly_dataframes['head.x'].values.astype(float)*calib_x#-tx
    hy=fly_dataframes['head.y'].values.astype(float)*calib_y#-ty

    # abdomen
    ax=fly_dataframes['abdomen.x'].values.astype(float)*calib_x#-tx
    ay=fly_dataframes['abdomen.y'].values.astype(float)*calib_y#-ty

    # l1
    l1_x=fly_dataframes['L1.x'].values.astype(float)*calib_x#-tx
    l1_y=fly_dataframes['L1.y'].values.astype(float)*calib_y#-ty

    # l2
    l2_x=fly_dataframes['L2.x'].values.astype(float)*calib_x#-tx
    l2_y=fly_dataframes['L2.y'].values.astype(float)*calib_y#-ty

    # l3
    l3_x=fly_dataframes['L3.x'].values.astype(float)*calib_x#-tx
    l3_y=fly_dataframes['L3.y'].values.astype(float)*calib_y#-ty

    # r1
    r1_x=fly_dataframes['R1.x'].values.astype(float)*calib_x#-tx
    r1_y=fly_dataframes['R1.y'].values.astype(float)*calib_y#-ty

    # r2
    r2_x=fly_dataframes['R2.x'].values.astype(float)*calib_x#-tx
    r2_y=fly_dataframes['R2.y'].values.astype(float)*calib_y#-ty

    # r3
    r3_x=fly_dataframes['R3.x'].values.astype(float)*calib_x#-tx
    r3_y=fly_dataframes['R3.y'].values.astype(float)*calib_y#-ty
    
#     raw_positions = -tx, -ty, -hx, -hy, -ax, -ay, -r1_x, -r1_y, -r2_x, -r2_y, -r3_x, -r3_y, -l1_x, -l1_y, -l2_x, -l2_y, -l3_x,-l3_y
    raw_positions = tx, ty, hx, hy, ax, ay, r1_x, r1_y, r2_x, r2_y, r3_x, r3_y, l1_x, l1_y, l2_x, l2_y, l3_x,l3_y
                     
    return frame_idx, raw_positions