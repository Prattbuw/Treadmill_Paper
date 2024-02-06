# Split belt motor control with PID control, video acquisition, closed-loop pose estimation
# Brandon Pratt. updated 12/23/2020

# Import libraries
from pypylon import pylon
from pypylon import genicam
import concurrent.futures
import nidaqmx
import numpy
import sys
import subprocess
import math
import msvcrt
import time
import warnings
import os
from imageio import get_writer
from Phidget22.Devices.DCMotor import *
from Phidget22.Devices.Encoder import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. Either add PhdiegtHelperFunctions.py to your project folder "
                      "or remove the import from your project.")
    sys.exit()

# Setup phidget motor microcontroller
warnings.filterwarnings("ignore", category = ResourceWarning)

'''
* Configures the device's DataInterval
* Displays info about the attached Phidget channel.
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param self The Phidget channel that fired the attach event
'''
def onAttachHandler(self):

    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information

        print("\nAttach Event:")

        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        else:
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel:  " + str(channel) + "\n")

        """
        * Set the DataInterval inside of the attach handler to initialize the device with this value.
        * DataInterval defines the minimum time between VelocityUpdate events...if it is desired to change the velocity
        * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
        """
        try:
            ph.setDataInterval(50) #adjust to desired value 0 assumes shortest interval
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Setting DataInterval: \n\t")
            DisplayError(e)
            return

    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
* Displays info about the detached Phidget channel.
* Fired when a Phidget channel with onDetachHandler registered detaches
*
* @param self The Phidget channel that fired the attach event
"""
def onDetachHandler(self):

    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information

        print("\nDetach Event:")

        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        else:
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel:  " + str(channel) + "\n")

    except PhidgetException as e:
        print("\nError in Detach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
Event handler that detects when the position changes
"""
# left motor
def onPositionChange(self, positionChange, timeChange, indexTriggered):
    #print("PositionChange: " + str(positionChange))
    #print("TimeChange: " + str(timeChange))
    cnts=positionChange
    GR=50+(801/895) # gear ratio
    r=5.1943 # radius
    distance=((cnts/360)*(2*r*math.pi))/GR # adjust for the gear ratio
    distance_adjust=distance*3.6
    vcurr=(distance_adjust/(timeChange/1000))
    vel_store_right.append(vcurr)
    # store time
    curr_time=time.time()-right_t_start
    right_belt_timestamp.append(curr_time)

    #print('right motor speed: ', vcurr, ' mm/s')
    #print("Current Velocity: ", vcurr, " mm/s")
    #print("Target Velocity: ", vgoal, " mm/s")

    # PID control with update handler
    pid(vcurr,vgoal,timeChange,integral,error,derivative,slope)
    return

"""
PID control function. Control on veloctiy
"""
def pid(vcurr,vgoal,timeChange,integral,error,derivative,slope):
    #print('vgoal: ', vgoal)
    dt=timeChange/1000
    errorlast=error
    error=vgoal-vcurr
    current_duty_cycle=ch.getTargetVelocity()
    # compute the duty cycle
    duty_cycle=(((Kp*error) + (Ki*integral)+ (Kd * derivative))/slope)+current_duty_cycle;
    #print("Prior Duty Cycle: ", duty_cycle)

    # bound the duty cycle
    if duty_cycle > 1:
        duty_cycle = 1.0
    elif duty_cycle < 0.10: # lower limit to keep the duty cycle and motor velocity linear
        duty_cycle=0.10
    else:
        integral = integral+(error * dt)
        derivative = (error - errorlast)/dt

    # change the motor speed
    ch.setTargetVelocity(duty_cycle)

    # display the duty cycle
    #print("Updated Duty Cycle: ", ch.getTargetVelocity())

# right motor
def onPositionChange1(self, positionChange, timeChange, indexTriggered):
    #print("PositionChange: " + str(positionChange))
    #print("TimeChange: " + str(timeChange))
    cnts=positionChange
    GR=50+(801/895) # gear ratio
    r=5.1943 # radius
    distance=((cnts/360)*(2*r*math.pi))/GR # adjust for the gear ratio
    distance_adjust=distance*3.6
    vcurr1=-(distance_adjust/(timeChange/1000))
    vel_store_left.append(vcurr1)
    # store time
    curr_time=time.time()-left_t_start
    left_belt_timestamp.append(curr_time)

    #print('left motor speed: ', vcurr1, ' mm/s')
    #print("Current Velocity: ", vcurr, " mm/s")
    #print("Target Velocity: ", vgoal, " mm/s")

    # PID control with update handler
    pid1(vcurr1,vgoal1,timeChange,integral1,error1,derivative1,slope)
    return

"""
PID control function. Control on veloctiy
"""
def pid1(vcurr1,vgoal1,timeChange,integral1,error1,derivative1,slope):
    #print('vgoal1: ', vgoal1)
    dt=timeChange/1000
    errorlast1=error1
    error1=vgoal1-vcurr1
    current_duty_cycle=ch1.getTargetVelocity()
    # compute the duty cycle
    duty_cycle=(((Kp*error1) + (Ki*integral1)+ (Kd * derivative1))/slope)+current_duty_cycle;
    #print("Prior Duty Cycle: ", duty_cycle)

    # bound the duty cycle
    if duty_cycle > 1:
        duty_cycle = 1.0
    elif duty_cycle < 0.10: # lower limit to keep the duty cycle and motor velocity linear
        duty_cycle=0.10
    else:
        integral1 = integral1+(error1 * dt)
        derivative1 = (error1 - errorlast1)/dt

    # change the motor speed
    ch1.setTargetVelocity(duty_cycle)

"""
* Writes Phidget error info to stderr.
* Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
*
* @param self The Phidget channel that fired the attach event
* @param errorCode the code associated with the error of enum type ph.ErrorEventCode
* @param errorString string containing the description of the error fired
"""
def onErrorHandler(self, errorCode, errorString):

    sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")


"""
* Creates, configures, and opens a DCMotor channel.
* Provides interface for controlling TargetVelocity of the DCMotor.
* Closes out DCMotor channel
*
* @return 0 if the program exits successfully, 1 if it exits with errors.
"""

"""
* Allocate a new Phidget Channel object
"""
try:
    # left motor
    ch = DCMotor()
    che = Encoder()

    # right motor
    ch1 = DCMotor()
    che1 = Encoder()
except PhidgetException as e:
    sys.stderr.write("Runtime Error -> Creating DCMotor: \n\t")
    DisplayError(e)
    raise
except RuntimeError as e:
    sys.stderr.write("Runtime Error -> Creating DCMotor: \n\t" + e)
    raise

"""
* Set matching parameters to specify which channel to open
"""
#You may remove this line and hard-code the addressing parameters to fit your application
#channelInfo = AskForDeviceParameters(ch)

ch.setDeviceSerialNumber=625034
ch.setHubPort=0#(channelInfo.hubPort)
ch.setIsHubPortDevice=False#(channelInfo.isHubPortDevice)
ch.setChannel=0#(channelInfo.channel)

ch1.setDeviceSerialNumber=624852
ch1.setHubPort=0#(channelInfo.hubPort)
ch1.setIsHubPortDevice=False#(channelInfo.isHubPortDevice)
ch1.setChannel=0#(channelInfo.channel)

"""
* Add event handlers before calling open so that no events are missed.
"""
# left motor
print("\n--------------------------------------")
print("\nSetting OnAttachHandler...")
ch.setOnAttachHandler(onAttachHandler)

print("Setting OnDetachHandler...")
ch.setOnDetachHandler(onDetachHandler)

print("Setting OnErrorHandler...")
ch.setOnErrorHandler(onErrorHandler)

# Register for event before calling open...event handler when position changes
che.setOnPositionChangeHandler(onPositionChange)

# right motor
print("\n--------------------------------------")
print("\nSetting OnAttachHandler...")
ch1.setOnAttachHandler(onAttachHandler)

print("Setting OnDetachHandler...")
ch1.setOnDetachHandler(onDetachHandler)

print("Setting OnErrorHandler...")
ch1.setOnErrorHandler(onErrorHandler)

# Register for event before calling open...event handler when position changes
che1.setOnPositionChangeHandler(onPositionChange1)


"""
* Open the channel with a timeout
"""
print("\nOpening and Waiting for Attachment...")

try:
    # left motor
    ch.openWaitForAttachment(5000)
    che.openWaitForAttachment(5000)

    # right motor
    ch1.openWaitForAttachment(5000)
    che1.openWaitForAttachment(5000)
except PhidgetException as e:
    PrintOpenErrorMessage(e, ch)
    raise EndProgramSignal("Program Terminated: Open Failed")

# set motor properties
# left motor
maxBrakingStrength = ch.getMaxBrakingStrength()
ch.setTargetBrakingStrength(maxBrakingStrength)

# right motor
maxBrakingStrength = ch1.getMaxBrakingStrength()
ch1.setTargetBrakingStrength(maxBrakingStrength)

# set encoder properties
# left motor
minDataInterval = che.getMinDataInterval()
#print("MinDataInterval: " + str(minDataInterval))
che.setDataInterval(32)

# right motor
minDataInterval = che1.getMinDataInterval()
#print("MinDataInterval: " + str(minDataInterval))
che1.setDataInterval(32)

# v_change=20# 1 mm/s change
Kp=0.75 # proportional gain coefficient
Ki=1 # integral gain coefficient
Kd=0.2 # derivative gain coefficient
integral=0 # initialize integral
error=0
derivative=0
integral1=0 # initialize integral
error1=0
derivative1=0
vel_store_left=[] # intialize
vel_store_right=[]
left_belt_timestamp=[] # time of belt speed update
left_t_start=time.time()
right_belt_timestamp=[]
right_t_start=time.time()
slope=200 # conversion between v (mm/s) to duty cycle. empircally calculated (same for left and right motor)
GR=50+(801/895) # gear ratio

# set the position
# left motor
che.setPosition(0)
che.setPositionChangeTrigger(1)# position must change by 1

# # right motor
che1.setPosition(0)
che1.setPositionChangeTrigger(1)# position must change by 1

# Start the motor in an ON state
# set starting velocity
duty_cycle=0.10#float(input('Duty Cycle:'))

if duty_cycle > 1.0:
    duty_cycle=1.0
if duty_cycle <0:
    duty_cycle=0
ch.setTargetVelocity(0) # initialize target velocity for first motor
ch1.setTargetVelocity(0) # initialize target velocity for first motor

'''
Motor Parameters
'''

#Specify input parameters
# Create new directory to save data
genotype = 'R48A07AD_kir/'#'jr688_kir/' ##'23Bb_ss04746_kir/' # keep slash
print('Create Directory for Data?..1. Yes  2. No')
mk_dir=int(input('Create Directory?'))
if mk_dir==1:
    dir_name=str(input('Directory_name:'))
    #dir_path='D:/Sarah/Data/Videos/'+dir_name
    dir_path='F:/Brandon/Data/split_belt/long_split/'+ genotype + dir_name+'/'
    #adjust for Pierre's anipose format
    #dir_path_vid='D:/Sarah/Data/Videos/'+dir_name+'/videos-raw'
    dir_path_vid='F:/Brandon/Data/split_belt/long_split/'+ genotype + dir_name+'/videos-raw'
    # Create target Directory if don't exist
    if not os.path.exists(dir_path_vid):
        os.mkdir(dir_path)
        os.mkdir(dir_path_vid)
        save_path=dir_path_vid+'/'
        print("Directory " , dir_path_vid,  " Created ")
    else:
        save_path=dir_path_vid+'/'
        print("Directory " , dir_path_vid ,  " already exists...save videos to this path")
else:
    print("Directory not created")
    #save_path='D:/Sarah/Data/Videos/'
    save_path='F:/Brandon/Data/split_belt/long_split' + genotype

#parameter loadout
fly_num=str(input('Fly number (#):'))

# specify trials and speed parameters
mod_factor = 1.725
v_base = 10 # presplit and postsplit driving speeds
mod= 0
mod1=0
v_mod = (v_base * mod_factor) + mod
v_mod1 = (v_base * mod_factor) + mod1
v_start = 6 # starting speed (mm/s)
v_start_mod = (v_start * mod_factor) + mod
v_start_mod1 = (v_start * mod_factor) + mod1
percent_change = 0.2 # max - 0.25 #percent change in driving speed
v_low = v_base - (v_base * percent_change)
v_high = v_base + (v_base * percent_change)
n_trials = 10 # number of trials per condition

# parameter for acquistion
fps = 200 # was 180
dt = 1/fps
ramp_slope = 1 # mm/s^2 for getting to speed
ttl_fps=fps #Match this to TTL frequency
trial_duration = 20 # second(s)...recording time
delay_time = 40 # time to delay between trials...takes into account the delay for grabbing frames
elapsed_time=0

# compute the total number of samples for the stimulus
tot_samples=trial_duration*fps
end_frame=tot_samples
IMAGES_TO_GRAB=end_frame #Set to double the end_frame just ensure that all frames will be captures
IMAGES_TO_GRAB=int(round(IMAGES_TO_GRAB))
exit=True #Set to true to begin aquisition session

'''
Function to save videos in parallel
'''
def save_video(cam_img):
    filename='fly'+fly_num+cond_name_rs+str(v_name_R)+cond_name_ls+str(v_name_L)+'_'+split_period+'_Trial_'+str(trial+1)
    with get_writer(save_path + filename + cam_img[0] +'.avi',fps=fps,codec='libx264', quality=None, bitrate=None, pixelformat='yuv420p',ffmpeg_params=['-preset','ultrafast','-crf','15', '-tune', 'zerolatency']) as writer:
        # write video
        for img in cam_img[1]:
            writer.append_data(img)

        #close writer
        writer.close()

    return cam_img[0] + ' saved'


    # writer=get_writer(
    #    save_path + filename + cam_img[0] +'.avi',  # .mp4, mkv players often support H.264, Camera1
    #     # test harddrive speed
    #    #'E:/Brandon_Test/'+ filename +'.avi',
    #     # use .avi (not .mp4) format because can be opened in virtualdub
    #     fps=fps,  # FPS is in units Hz; should be real-time...playback...set to actual
    #     codec='libx264',  # When used properly, this is basically
    #                       # "PNG for video" (i.e. lossless)
    #     quality=None,  # disables variable compression...0 to 10
    #     bitrate=None, #1000000, # set bit rate
    #     pixelformat='yuv420p',  # widely used
    #     ffmpeg_params=['-preset','ultrafast','-crf','15', '-tune', 'zerolatency'], # crf:0-51 lower equals more quality
    #     input_params=None
    # )


'''
function to capture frames in parallel
'''
def capture_frames(cam_objs):

    wait_timeout=7500 #200 # amount of time alloted before a timeout occurs.Can't be zero
    #cam_objs[1].WaitForFrameTriggerReady(wait_time)
    # result = cam_objs[1].RetrieveResult(timeout,pylon.TimeoutHandling_ThrowException)
    result = cam_objs[1].RetrieveResult(wait_timeout,pylon.TimeoutHandling_ThrowException)

    # FOR TESTING camera alignment
    # if (result.NumberOfSkippedImages > 0):
    #     print(result.NumberOfSkippedImages)

    if result.GrabSucceeded():
        grab_val = 0
    else:
        grab_val = 1
        # print("Error: ", result.GetErrorCode(), result.GetErrorDescription())

    # release camera result so that iut can capture the next one
    #result.Release()

    return [cam_objs[0], result.Array, grab_val, result]


while exit == True: #True

    # camera setting and initialization
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    if len(devices) == 0:
        raise pylon.RUNTIME_EXCEPTION("No camera present.")

    # locate and specify camera1
    camera1 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[0]))

    camera1.Open()

        # locate and specify camera2
    camera2 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[2]))

    camera2.Open()

        # camera 3
            # locate and specify camera3
    camera3 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[3]))

    camera3.Open()

        # camera 4
            # locate and specify camera3
    camera4 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[4]))

    camera4.Open()

    # camera 5
            # locate and specify camera3
    camera5 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[1]))

    camera5.Open()

    """
    setup camera settings
    """

   #SET PARAMETERS FOR CAMERA 1

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera1.MaxNumBuffer = 100
    #camera.Width = camera.Width.Max
    camera1.Width = 736
    #camera.Height = camera.Height.Max
    camera1.Height = 496
    exposure_time=2000# in microseconds...2000
    camera1.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)

    try:
        camera1.Gain = camera1.Gain.Max
    except genicam.LogicalErrorException:
        camera1.GainRaw = camera1.GainRaw.Max

    # Maximize the Image AOI.
    if genicam.IsWritable(camera1.OffsetX):
        #camera1.OffsetX = camera1.OffsetX.Min
        camera1.OffsetX = 48
    if genicam.IsWritable(camera1.OffsetY):
        #camera1.OffsetY = camera1.OffsetY.Min
        camera1.OffsetY = 126

    camera1.PixelFormat = "Mono8"

    camera1.AcquisitionFrameRate=camera1.ResultingFrameRate()

    print('Camera 1 resulting fps =',camera1.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 2

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera2.MaxNumBuffer = 100
    #camera.Width = camera.Width.Max
    camera2.Width = 736
    #camera.Height = camera.Height.Max
    camera2.Height = 432
    camera2.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    try:
        camera2.Gain = camera2.Gain.Max
    except genicam.LogicalErrorException:
        camera2.GainRaw = camera2.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera2.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera2.OffsetX = 80
    if genicam.IsWritable(camera2.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera2.OffsetY = 135

    camera2.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera2.AcquisitionFrameRate=camera2.ResultingFrameRate()
    print('Camera 2 resulting fps =',camera2.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 3

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera3.MaxNumBuffer = 100
    #camera.Width = camera.Width.Max
    camera3.Width = 768
    #camera.Height = camera.Height.Max
    camera3.Height = 448
    camera3.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    try:
        camera3.Gain = camera3.Gain.Max
    except genicam.LogicalErrorException:
        camera3.GainRaw = camera3.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera3.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera3.OffsetX = 32
    if genicam.IsWritable(camera3.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera3.OffsetY = 124

    camera3.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera3.AcquisitionFrameRate=camera3.ResultingFrameRate()
    print('Camera 3 resulting fps =',camera3.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 4

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera4.MaxNumBuffer = 100
    #camera.Width = camera.Width.Max
    camera4.Width = 736
    #camera.Height = camera.Height.Max
    camera4.Height = 368
    camera4.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera4.Gain = camera4.Gain.Max
    except genicam.LogicalErrorException:
        camera4.GainRaw = camera4.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera4.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera4.OffsetX = 16
    if genicam.IsWritable(camera4.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera4.OffsetY = 214

    camera4.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera4.AcquisitionFrameRate=camera4.ResultingFrameRate()
    print('Camera 4 resulting fps =',camera4.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 5

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera5.MaxNumBuffer = 100
    #camera.Width = camera.Width.Max
    camera5.Width = 496
    #camera.Height = camera.Height.Max
    camera5.Height = 608
    camera5.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera5.Gain = camera5.Gain.Max
    except genicam.LogicalErrorException:
        camera5.GainRaw = camera5.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera5.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera5.OffsetX = 144
    if genicam.IsWritable(camera5.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera5.OffsetY = 16

    camera5.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera5.AcquisitionFrameRate=camera4.ResultingFrameRate()
    print('Camera 5 resulting fps =',camera4.ResultingFrameRate(),'Hz')

    # Perform continous aquisition in order to set initial condition
    # Configure the trigger acquistion camera1
    # Send signal out when exposure is active on Line4
    camera1.LineSelector= "Line4"
    camera1.LineMode= "Output"
    camera1.LineInverter= False
    camera1.LineSource="ExposureActive"

    #Set Line 3 to trigger
    camera1.LineSelector="Line3"
    camera1.LineMode="Input"
    camera1.TriggerSelector="FrameStart"
    camera1.TriggerMode="On"
    camera1.TriggerSource="Line3"
    camera1.TriggerActivation="RisingEdge"
    camera1.TriggerDelay=0

    # Configure the trigger acquistion camera 2
    # Send signal out when exposure is active on Line4
    camera2.LineSelector= "Line4"
    camera2.LineMode= "Output"
    camera2.LineInverter= False
    camera2.LineSource="ExposureActive"

    #Set Line 3 to trigger
    camera2.LineSelector="Line3"
    camera2.LineMode="Input"
    camera2.TriggerSelector="FrameStart"
    camera2.TriggerMode="On"
    camera2.TriggerSource="Line3"
    camera2.TriggerActivation="RisingEdge"
    camera2.TriggerDelay=0

    # Configure the trigger acquistion camera 3

    # Send signal out when exposure is active on Line4
    camera3.LineSelector= "Line4"
    camera3.LineMode= "Output"
    camera3.LineInverter= False
    camera3.LineSource="ExposureActive"

    #Set Line 3 to trigger
    camera3.LineSelector="Line3"
    camera3.LineMode="Input"
    camera3.TriggerSelector="FrameStart"
    camera3.TriggerMode="On"
    camera3.TriggerSource="Line3"
    camera3.TriggerActivation="RisingEdge"
    camera3.TriggerDelay=0

    # Send signal out when exposure is active on Line4
    camera4.LineSelector= "Line4"
    camera4.LineMode= "Output"
    camera4.LineInverter= False
    camera4.LineSource="ExposureActive"

    #Set Line 4 to trigger
    camera4.LineSelector="Line3"
    camera4.LineMode="Input"
    camera4.TriggerSelector="FrameStart"
    camera4.TriggerMode="On"
    camera4.TriggerSource="Line3"
    camera4.TriggerActivation="RisingEdge"
    camera4.TriggerDelay=0

    # Send signal out when exposure is active on Line4
    camera5.LineSelector= "Line4"
    camera5.LineMode= "Output"
    camera5.LineInverter= False
    camera5.LineSource="ExposureActive"

    #Set Line 4 to trigger
    camera5.LineSelector="Line3"
    camera5.LineMode="Input"
    camera5.TriggerSelector="FrameStart"
    camera5.TriggerMode="Off" # start off in the off state
    camera5.TriggerSource="Line3"
    camera5.TriggerActivation="RisingEdge"
    camera5.TriggerDelay=0

    # randomize condition type
    rand_cond = numpy.random.permutation(2)
    cond_state = 0


    '''

    START TRIAL
    '''
    lr_cond_cnt = -1
    for cond in rand_cond: # left or right split
        lr_cond_cnt += 1
        rand_split_speed_array=numpy.zeros((2,n_trials)) # just for split
        if cond == 0:
            # prescribe speed conditions for left and right belts during split
            cond_name_rs = '_RS_L_'
            cond_name_ls = '_LS_H_'
            rand_split_speed_array[0, :] = v_low
            rand_split_speed_array[1, :] = v_high
        else:
            cond_name_rs = '_RS_H_'
            cond_name_ls = '_LS_L_'
            rand_split_speed_array[0, :] = v_high
            rand_split_speed_array[1, :] = v_low
        rand_split_speed_array = rand_split_speed_array.astype(int)

        # iterate through split periods
        trial_state = 0
        period_names = ['presplit', 'split', 'postsplit']
        num_trials = [n_trials,n_trials,n_trials]

        for period in range(3):
            split_period =  period_names[period]

            for trial in range(num_trials[period]):
                # specify split condition
                if period == 1:
                    v_name_R = rand_split_speed_array[0, trial]
                    v_name_L = rand_split_speed_array[1, trial]

                    if v_name_R > v_name_L:
                        v_change_right = v_mod + (v_mod * percent_change)
                        v_change_left = v_mod1 - (v_mod1 * percent_change)
                    else:
                        v_change_right = v_mod - (v_mod * percent_change)
                        v_change_left = v_mod1 + (v_mod1 * percent_change)

                else:
                    v_name_L = v_base
                    v_name_R = v_base
                    v_change_right = v_mod
                    v_change_left = v_mod1

                #specify filename
                filename='fly'+fly_num+cond_name_rs+str(v_name_R)+cond_name_ls+str(v_name_L)+'_'+split_period+'_Trial_'+str(trial+1)

                if (trial == 0) and (split_period == 'presplit') and (cond_state == 0):
                    # initialize belt at starting speed
                    cond_state = 1
                    vgoal = v_start_mod
                    vgoal1 = v_start_mod1
                    ch1.setTargetVelocity(duty_cycle)
                    ch.setTargetVelocity(duty_cycle)

                    # display top-down camera 5
                    camera5.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                    imageWindow_cam5 = pylon.PylonImageWindow()
                    imageWindow_cam5.Create(5)

                    print('Trigger to record')
                    Wait = 3000

                    # Perform pose estimation and determine if the imposed threshold was surpassed
                    img_count = 0

                    # set camera 5 in non-trigger mode
                    while True:
                        #try:
                        # break by checking for a key press
                        if msvcrt.kbhit():
                            if msvcrt.getwche() == '\r':
                                break

                        result_camera5=camera5.RetrieveResult(
                                Wait,
                                pylon.TimeoutHandling_ThrowException)


                        if result_camera5.GrabSucceeded():

                            #Display option
                            imageWindow_cam5.SetImage(result_camera5)
                            imageWindow_cam5.Show()

                    #stop grabbing
                    print('Triggered')
                    imageWindow_cam5.Close()
                    camera5.StopGrabbing()

                    # set camera 5 to trigger mode
                    camera5.TriggerMode="On"

                # wait period so that timing is accurate
                if trial_state > 0:
                    print("Delay Start")

                    # add a time delay that allows for a rest period and enough time for the video to be saved
                    d_time=time.time()-t_delay
                    print('Delay time= ',d_time, ' seconds')
                    total_delay_time=delay_time-(time.time()-t_delay)
                    delay_start=time.time()
                    time.sleep(float(total_delay_time))
                    total_elapsed_time=time.time() -delay_start
                    print("Delay End")
                    print('Delayed for ',total_elapsed_time, 'seconds')
                    #append rest period to meta data
                    rest_period=total_elapsed_time+d_time

                try:
                    #place images into a buffer and intialize variables
                    # wait_time=2500 #5000 for 200Hz
                    frame_count=0 # reset frame count
                    buffer=[]
                    buffer_camera2=[]
                    buffer_camera3=[]
                    buffer_camera4=[]
                    buffer_camera5=[]
                    timestamp=[]
                    left_belt_timestamp=[] # time of belt speed update
                    left_t_start=time.time()
                    right_belt_timestamp=[]
                    right_t_start=time.time()
                    vel_store_left=[] # store the velocity of the left and right belts
                    vel_store_right=[]
                    stimulus=[] #record the velocity of the belt
                    stimulus1=[] #record the velocity of the belt

                    # combine the camera objects for multi-threading
                    cam_objs=[['cam1', camera1], ['cam2',camera2], ['cam3', camera3], ['cam4', camera4], ['cam5', camera5]]
                    start_cam_id = 0 # set to one to signify that cameras were started
                    while frame_count<end_frame:

                        # multi-thread to start cameras
                        if start_cam_id == 0:
                            # with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
                            #     executor.map(start_cameras, cam_objs)
                            for c_start in cam_objs:
                                c_start[1].StartGrabbing(pylon.GrabStrategy_OneByOne)

                            # create and start thread to trigger camera
                            time.sleep(5) # delay to allow cameras to warm up

                            # thread pool to start trigger signal
                            # with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                            #     trigger_statement = executor.map(trigger_sig, out)
                            # print(trigger_statement)
                            #start subprocess to trigger cameras
                            sub_p = subprocess.Popen(['python', 'subprocess_daq_trigger.py'])
                            start_cam_id = 1

                            # # # setup threading task
                            # task = nidaqmx.Task()
                            # task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
                            # task.timing.cfg_samp_clk_timing(dac_rate, samps_per_chan=len(out)) #sample_mode=AcquisitionType.FINITE,
                            # w_thread = threading.Thread(target=trigger_sig, args=(out,))
                            # w_thread.start() # start triggering cameras

                            # change the start camera id so that frames start to be captured

                        else:

                            '''
                            Multi-threaded grabbing approach
                            '''

                            # acquire frames
                            with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
                                cam_results = executor.map(capture_frames, cam_objs)
                                camera_names = []
                                frame_results = []
                                grab_results = []
                                result_objs = []
                                for cam_info in cam_results:
                                    camera_names.append(cam_info[0])
                                    frame_results.append(cam_info[1])
                                    grab_results.append(cam_info[2])
                                    result_objs.append(cam_info[3])

                             #Image grabbed successfully?
                            if numpy.sum(numpy.array(grab_results)) == 0:

                                # record time
                                curr_time=float(time.time())
                                timestamp.append(curr_time)

                                #record stimulus velocity (right)
                                stimulus.append(ch.getVelocity())
                                stimulus1.append(ch1.getVelocity())

                                # append buffer of cameras
                                buffer.append(frame_results[0])
                                buffer_camera2.append(frame_results[1])
                                buffer_camera3.append(frame_results[2])
                                buffer_camera4.append(frame_results[3])
                                buffer_camera5.append(frame_results[4])

                                # Update frame counter
                                frame_count=frame_count+1

                                # check and update belt speed
                                # update the speed of the right belt by change vgoal during pertubation period
                                if (trial == 0) and (split_period == 'presplit'):
                                    if lr_cond_cnt == 0:
                                        if frame_count <= round(((v_mod - v_start_mod)/ramp_slope)*fps):
                                            vgoal = v_start_mod + (ramp_slope*(dt * frame_count))
                                            vgoal1 = v_start_mod1 + (ramp_slope*(dt * frame_count))
                                        else:
                                            vgoal = v_change_right
                                            vgoal1 = v_change_left
                                    else:
                                        vgoal = v_change_right
                                        vgoal1 = v_change_left
                                elif (trial == 1) and (split_period == 'presplit'):
                                    vgoal = v_change_right
                                    vgoal1 = v_change_left
                                elif (trial == 1) and (split_period == 'postsplit'):
                                    vgoal = v_change_right
                                    vgoal1 = v_change_left
                                elif (trial == 0) and (split_period == 'postsplit'):
                                    if frame_count <= round(((v_mod * percent_change)/ramp_slope)*fps):
                                        if rand_split_speed_array[0, -1] > rand_split_speed_array[1, -1]:
                                            vgoal = (v_mod + (v_mod * percent_change)) + (ramp_slope*(dt * frame_count)) # switch the signs here?
                                            vgoal1 = (v_mod1 + (v_mod1 * percent_change)) - (ramp_slope*(dt * frame_count))
                                        else:
                                            vgoal = (v_mod + (v_mod * percent_change)) - (ramp_slope*(dt * frame_count))
                                            vgoal1 = (v_mod1 + (v_mod1 * percent_change)) + (ramp_slope*(dt * frame_count))
                                    else:
                                        vgoal = v_change_right
                                        vgoal1 = v_change_left
                                elif (trial == 0) and (split_period == 'split'):
                                    if frame_count <= round(((v_mod * percent_change)/ramp_slope)*fps):
                                        if v_change_right > v_change_left:
                                            vgoal = v_mod + (ramp_slope*(dt * frame_count))
                                            vgoal1 = v_mod1 - (ramp_slope*(dt * frame_count))
                                        else:
                                            vgoal = v_mod - (ramp_slope*(dt * frame_count))
                                            vgoal1 = v_mod1 + (ramp_slope*(dt * frame_count))
                                    else:
                                        vgoal = v_change_right
                                        vgoal1 = v_change_left
                                elif split_period == 'split':
                                    vgoal = v_change_right
                                    vgoal1 = v_change_left


                            # release camera results
                            for r in result_objs:
                                r.Release()

                    # # stop grabbing images
                    camera1.StopGrabbing()
                    camera2.StopGrabbing()
                    camera3.StopGrabbing()
                    camera4.StopGrabbing()
                    camera5.StopGrabbing()

                    # # join thread
                    # w_thread.join()

                    # end subprocess
                    # time.sleep(5)
                    sub_p.wait(timeout = 1)
                    sub_p.kill()

                    # Isolate only the belt updates (speed and time) during the recording session
                    t_delay=time.time()
                    trial_state = 1
                    right_belt_time=right_belt_timestamp
                    right_belt_speed=vel_store_right
                    left_belt_time=left_belt_timestamp
                    left_belt_speed=vel_store_left

                    # treadmill belt speed update time
                    l_belt_timestamp=numpy.array(left_belt_time)
                    r_belt_timestamp=numpy.array(right_belt_time)
                    numpy.savetxt(save_path + filename+'_Left_Belt_TimeStamps.txt',l_belt_timestamp,delimiter=',')
                    numpy.savetxt(save_path + filename+'_Right_Belt_TimeStamps.txt',r_belt_timestamp,delimiter=',')

                    #save stimulus array
                    stimulus=numpy.array(left_belt_speed)
                    stimulus1=numpy.array(right_belt_speed)

                    # save converted numpy array as text file
                    numpy.savetxt(save_path + filename+'_Stimulus_Left_Belt_Speed.txt',stimulus,delimiter=',')
                    numpy.savetxt(save_path + filename+'_Stimulus_Right_Belt_Speed.txt',stimulus1,delimiter=',')

                    print('Saving Videos')
                    # save videos using parallel processing
                    cam_imgs=[['Camera1', buffer], ['Camera2', buffer_camera2], ['Camera3', buffer_camera3], ['Camera4', buffer_camera4], ['Camera5', buffer_camera5]]

                    def main():
                        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
                            results= executor.map(save_video,cam_imgs)

                            for result in results:
                                print(result) # displays when cameras are saved
                    if __name__ == '__main__':
                        main()

                    #save timestamp array
                    camera_timestamp=numpy.array(timestamp)
                    camera_timestamp=camera_timestamp-camera_timestamp[0]
                    #print('length of timestamp array', len(timestamp))
                    numpy.savetxt(save_path + filename+'_Camera_TimeStamps.txt',camera_timestamp,delimiter=',')
                    print('Finsihed Saving Repeat')

                except genicam.GenericException as e:
                    # Error handling.
                    print("An exception occurred.", e.GetDescription())
                    #exitCode = 1

    # close phidget channel
    ch.close()
    ch1.close()


    # camera has to be closed manually
    camera1.Close()
    camera2.Close()
    camera3.Close()
    camera4.Close()
    camera5.Close()

    exit=False
