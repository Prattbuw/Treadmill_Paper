import numpy as np 
import math 

def align_allocentric(norm_x_pos, norm_y_pos, norm_body_pos):

    # Align fly to heading
    r1_x = norm_x_pos[0]
    r1_y = norm_y_pos[0]

    r2_x = norm_x_pos[1]
    r2_y = norm_y_pos[1] 

    r3_x = norm_x_pos[2]
    r3_y = norm_y_pos[2] 

    l1_x = norm_x_pos[3]
    l1_y = norm_y_pos[3]

    l2_x = norm_x_pos[4]
    l2_y = norm_y_pos[4] 

    l3_x = norm_x_pos[5]
    l3_y = norm_y_pos[5]

    hx=norm_body_pos[0]
    hy=norm_body_pos[1]

    tx = norm_body_pos[2] 
    ty = norm_body_pos[3] 

    ax=norm_body_pos[4]
    ay=norm_body_pos[5]


    # compute heading angle
    htx = hx # normalize head x to thorax x 
    hty = hy # normalize head y to thorax y 

    heading_angle=np.zeros(len(htx))
    for j in range(len(htx)):
        # compute heading angle
        if htx[j]<0:

            # adjust for 180 degrees
            sign=1
            l_head=sign*math.sqrt(htx[j]**2 + hty[j]**2) # distance the head is away from the thorax
            angle=math.asin(hty[j]/l_head) # headin angle in radians
            if angle<0:
                heading_angle[j]=math.degrees((math.pi + angle))
            else:
                heading_angle[j]=math.degrees((angle - math.pi))

        else:
            sign=-1
            l_head=sign*math.sqrt(htx[j]**2 + hty[j]**2) # distance the head is away from the thorax
            heading_angle[j]=math.degrees(math.asin(hty[j]/l_head)) # headin angle in deg
            
    rot_vel = np.diff(heading_angle) 



    # shift leg and body points to common heading angle (0,0) 
    # store variables

    n_frames=len(hx)
    new_hx=np.zeros(n_frames)
    new_hy=np.zeros(n_frames)
    new_ax=np.zeros(n_frames)
    new_ay=np.zeros(n_frames)
    new_r1_x=np.zeros(n_frames)
    new_r1_y=np.zeros(n_frames)
    new_r2_x=np.zeros(n_frames)
    new_r2_y=np.zeros(n_frames)
    new_r3_x=np.zeros(n_frames)
    new_r3_y=np.zeros(n_frames)
    new_l1_x=np.zeros(n_frames)
    new_l1_y=np.zeros(n_frames)
    new_l2_x=np.zeros(n_frames)
    new_l2_y=np.zeros(n_frames)
    new_l3_x=np.zeros(n_frames)
    new_l3_y=np.zeros(n_frames)

    # xr = np.zeros(len(tx))
    # yr = np.zeros(len(tx))
    shift_angle = np.zeros(len(tx))

    for j in range(len(tx)):

        shift_angle = math.radians(heading_angle[j])

        # Rotation matrix multiplication to get rotated x & y
        new_hx[j] = ((hx[j]) * math.cos(shift_angle)) - ((hy[j]) * math.sin(shift_angle))
        new_hy[j] = ((hx[j]) * math.sin(shift_angle)) + ((hy[j])* math.cos(shift_angle)) 

        new_ax[j] = ((ax[j]) * math.cos(shift_angle)) - ((ay[j]) * math.sin(shift_angle))
        new_ay[j] = ((ax[j]) * math.sin(shift_angle)) + ((ay[j])* math.cos(shift_angle)) 

        new_r1_x[j] = ((r1_x[j]) * math.cos(shift_angle)) - ((r1_y[j]) * math.sin(shift_angle))
        new_r1_y[j] = ((r1_x[j]) * math.sin(shift_angle)) + ((r1_y[j])* math.cos(shift_angle)) 

        new_r2_x[j] = ((r2_x[j]) * math.cos(shift_angle)) - ((r2_y[j]) * math.sin(shift_angle))
        new_r2_y[j] = ((r2_x[j]) * math.sin(shift_angle)) + ((r2_y[j])* math.cos(shift_angle)) 

        new_r3_x[j] = ((r3_x[j]) * math.cos(shift_angle)) - ((r3_y[j]) * math.sin(shift_angle))
        new_r3_y[j] = ((r3_x[j]) * math.sin(shift_angle)) + ((r3_y[j])* math.cos(shift_angle)) 

        new_l1_x[j] = ((l1_x[j]) * math.cos(shift_angle)) - ((l1_y[j]) * math.sin(shift_angle))
        new_l1_y[j] = ((l1_x[j]) * math.sin(shift_angle)) + ((l1_y[j])* math.cos(shift_angle)) 

        new_l2_x[j] = ((l2_x[j]) * math.cos(shift_angle)) - ((l2_y[j]) * math.sin(shift_angle))
        new_l2_y[j] = ((l2_x[j]) * math.sin(shift_angle)) + ((l2_y[j])* math.cos(shift_angle)) 

        new_l3_x[j] = ((l3_x[j]) * math.cos(shift_angle)) - ((l3_y[j]) * math.sin(shift_angle))
        new_l3_y[j] = ((l3_x[j]) * math.sin(shift_angle)) + ((l3_y[j])* math.cos(shift_angle)) 


    # store leg arrays
    new_x_pos=[new_r1_x, new_r2_x, new_r3_x, new_l1_x, new_l2_x, new_l3_x]
    new_y_pos=[new_r1_y, new_r2_y, new_r3_y, new_l1_y, new_l2_y, new_l3_y]

    # store rotated body arrays
    rot_head=[new_hx, new_hy]
    rot_abdomen=[new_ax, new_ay]
    
    return heading_angle, rot_vel, new_x_pos, new_y_pos, rot_head, rot_abdomen 
          