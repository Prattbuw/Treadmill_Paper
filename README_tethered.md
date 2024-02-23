#### Overview

This dataset contains kinematics of wild-type Berlin fruit flies walking on an air-cushioned ball (N=12 flies). Videos from 6 different camera angles were recorded at 300 FPS, with tracked 2D keypoints on each joint for each leg, and then triangulated into 3D kinematics using Anipose. However, analysis of joint kinematics is still ongoing work and not included in this manuscript. The behavioral classifier from the Anipose paper (Karashchuk et. al, 2021) was used to distinguish bouts of walking (and of other behaviors). We also tracked the movement of the ball using FicTrac (Moore et. al, 2014), which serves as a readout for the forward and rotational walking velocity of the fly. Experiments were performed by Grant Chou on 09/14/2023 and 09/19/2023.  

#### Body points
We tracked the following keypoints along the body of the fly:
- abdomen-tip: the tip of the abdomen
- stripe-3: 3rd abdomen stripe from the top
- stripe-1: 1st abdomen stripe from the top
- thorax-abdomen: thorax-abdomen connective
- head-thorax: head-thorax (neck) connective
- l-eye-t: top of the left eye
- l-eye-b: bottom of the left eye
- r-eye-t: top of the right eye
- r-eye-b: bottom of the right eye

The 3D positions of these body points are stored as follows:
- x y z - the relevant coordinate axis (e.g. "abdomen-tip_x" is the position of abdomen-tip along x-axis)

The body lengths of each fly were calculated by summing the 3D distances between abdomen-tip to thorax-abdomen, thorax-abdomen to head-thorax, and head-thorax to the midpoint between l-eye-t and r-eye-t.

#### Leg joints

Flies have 6 legs, denoted as follows:
- L1 - front left leg
- L2 - mid left leg
- L3 - hind left leg
- R1 - front right leg
- R2 - mid right leg
- R3 - hind right leg

We track 5 points on each leg, denoted A-E:
- A - body-coxa joint
- B - coxa-femur joint (this point is roughly positioned at the trochanter)
- C - femur-tibia joint
- D - tibia-tarsus joint
- E - tarsus tip

Each joint is uniquely specified by the leg and joint name. For instance, L1A would be the left front leg body-coxa joint.

The 3D positions of the legs are stored for each joint leg combination as follows:
- x y z - the relevant coordinate axis (e.g. "L1A_x" is the position of L1A along x-axis)

Note also that due to the way we perform camera calibration, the scale of the x y z coordinates is in arbitrary units.
If you want to get units of mm, we recommend scaling the coordinates such that the length of the left leg
coxa ||L1A - L1B|| is 0.456 mm, our empirical measurement of the average coxa length for these flies.

The x y z axes are not arbitrary. We rotate the coordinate frame according to the following orthogonal axes:
- x axis goes L1A -> R1A
- y axis roughly goes L3A -> L1A, but orthogonalized with x axis
- z axis is cross product of x and y axes, roughly down -> up

#### Angles (not analyzed in this manuscript)
There are three types of angles:
- {leg}{joint}_rot: rotation about the axis of the leg
- {leg}{joint}_flex: flexion of the joint
- {leg}A_abduct: abduction of the joint (only for the body-coxa joint)

For example the L2B_rot column is the middle left leg femur rotation angle and the R1C_flex column is the front right leg femur-tibia flexion angle.  

We've also computed Hilbert transform phases for each joint angle and tarsus tip, as well as the 1st and 2nd-order derivatives of each joint angle, denoted respectively as:
- {leg}{joint}_{angle or tarsus tip coordinate}_phase
- {leg}{joint}_{angle}_d1
- {leg}{joint}_{angle}_d2

(e.g. R3E_y_phase = phase of the hind right tarsus tip y-coordinate, L1A_flex_d1 = 1st derivative of the front left leg body-coxa flexion angle)

We use the 3D positions of the legs to infer the angles of the fly legs.
In order to identify the angles in a unique way, we use a simple model for the leg, with the following assumptions:
1. only the body-coxa joint can abduct
2. all limbs can rotate, except the tarsus
3. all joints except body-coxa are restricted with up to 180 degrees of flexion

Refer to the attached diagram (leg_angles.png) to see the possible angles.

The angles are then encoded, in degrees, according to each joint and angle type.
A rotation of a limb (e.g. femur) is thought of as a rotation of the preceding joint (e.g. coxa-femur or B joint). 

The rotation (and abduction) angles wrap around 180 degrees, which can add discontinuities to the data.
We found we can get them to be continuous most of the time with this simple code:
r[r < 0] += 360
.. where r is an array containing a rotation angle.
The raw, uncorrected angles in the dataset, are denoted as
- {leg}{joint}_{angle}_raw

#### FicTrac
FicTrac calculates 3 axes of velocity of the ball
- fictrac_delta_rot_lab_x_mms: sideslip velocity in millimeters per second (i.e. sidestepping)
- fictrac_delta_rot_lab_y_mms: forward velocity in millimeters per second (i.e. forward walking)
- fictrac_delta_rot_lab_z_mms: rotational velocity in millimeters per second (i.e. turning)
- fictrac_delta_rot_lab_x_deg/s: sideslip velocity in degrees per second
- fictrac_delta_rot_lab_y_deg/s: forward velocity in degrees per second
- fictrac_delta_rot_lab_z_deg/s: rotational velocity in degrees per second 
  (I usually analyze forward velocity in mm/s and turning in deg/s)

as well as the integrated path (i.e. a flatmap of the fly's fictive path)
- fictrac_int_x_mm
- fictrac_int_y_mm

#### Other data

Other leg parameters
- {leg}_swing_stance, where swing = 0 and stance = 1
- {leg}_smoothed_velo: the 3d velocity of the tarsus tip (this was used as a thresholding parameter for the swing/stance classifier)

Behavioral classification is denoted as
- {behavior}_bout_number: a number identifying each bout of the behavior
- {behavior}_prob: the probability of the behavior (only one behavior classified per frarme, so highest probabilty "wins")

We're most interested in walking (i.e. 'walking_bout_number'), but other behaviors include standing, different types of grooming, and ball pushes. 

Experimental variables
- fullfile: the file name of the video (which you can find on flyviz.biz)
- fnum: the frame number from the video
- flyid: a unique identifier for the fly (formatted as '{mm}.{dd}.{yy} Fly {flynum}_{trialnum}', where the fly number is 1-indexed and the trial number is 0-indexed)