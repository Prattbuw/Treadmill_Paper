# Split belt motor control with PID control, video acquisition, closed-loop pose estimation
# Brandon Pratt. updated 12/23/2020

# Import libraries
from pypylon import pylon
from pypylon import genicam
import concurrent.futures
import numpy
import sys
import math
import msvcrt
import time
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
    GR=50 # gear ratio
    r=13.5001 # radius
    distance=((cnts/360)*(2*r*math.pi))/GR # adjust for the gear ratio
    distance_adjust=distance*(7/8)
    vcurr=-(distance_adjust/(timeChange/1000))
    vel_store.append(vcurr)
    # store time
    curr_time=time.time()-t_start
    belt_timestamp.append(curr_time)

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
    elif duty_cycle < 0.13: # lower limit to keep the duty cycle and motor velocity linear
        duty_cycle=0.13
    else:
        integral = integral+(error * dt)
        derivative = (error - errorlast)/dt

    # change the motor speed
    ch.setTargetVelocity(duty_cycle)

    # display the duty cycle
    #print("Updated Duty Cycle: ", ch.getTargetVelocity())

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
    ch = DCMotor()
    che = Encoder()
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

ch.setDeviceSerialNumber=479135#(channelInfo.deviceSerialNumber)
ch.setHubPort=0#(channelInfo.hubPort)
ch.setIsHubPortDevice=False#(channelInfo.isHubPortDevice)
ch.setChannel=0#(channelInfo.channel)

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

"""
* Open the channel with a timeout
"""
print("\nOpening and Waiting for Attachment...")

try:
    # left motor
    ch.openWaitForAttachment(5000)
    che.openWaitForAttachment(5000)

except PhidgetException as e:
    PrintOpenErrorMessage(e, ch)
    raise EndProgramSignal("Program Terminated: Open Failed")

# set motor properties
maxBrakingStrength = ch.getMaxBrakingStrength()
ch.setTargetBrakingStrength(maxBrakingStrength)

# set encoder properties
minDataInterval = che.getMinDataInterval()
#print("MinDataInterval: " + str(minDataInterval))
che.setDataInterval(32)

# v_change=20# 1 mm/s change
Kp=0.75 # proportional gain coefficient
Ki=1 # integral gain coefficient
Kd=0.2 # derivative gain coefficient
integral=0 # initialize integral
error=0
derivative=0
vel_store=[] # intialize
belt_timestamp=[] # time of belt speed update
t_start=time.time()
slope=70 # conversion between v (mm/s) to duty cycle. empircally calculated (same for left and right motor)
GR=50 # gear ratio

# set the position

che.setPosition(0)
che.setPositionChangeTrigger(1)# position must change by 1

# Start the motor in an ON state
# set starting velocity
duty_cycle=0.15#float(input('Duty Cycle:'))

if duty_cycle > 1.0:
    duty_cycle=1.0
if duty_cycle <0:
    duty_cycle=0
ch.setTargetVelocity(0) # initialize target velocity for first motor

#Specify input parameters
# Create new directory to save data
print('Create Directory for Data?..1. Yes  2. No')
mk_dir=int(input('Create Directory?'))
if mk_dir==1:
    dir_name=str(input('Directory_name:'))
    dir_path='F:/Brandon/Data/linear_belt/speed_dependent/52A01_DBD_tnt/'+dir_name+'/'
    #adjust for Pierre's anipose format
    #dir_path_vid='D:/Sarah/Data/Videos/'+dir_name+'/videos-raw'
    dir_path_vid='F:/Brandon/Data/linear_belt/speed_dependent/52A01_DBD_tnt/'+dir_name+'/videos-raw'
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
    save_path='F:/Brandon/Data/linear_belt/speed_dependent/52A01_DBD_tnt'

#parameter loadout
fly_num=str(input('Fly number (#):'))

# specify trials and speed parameters
v_base = 5 # 5 mm/s...baseline driving speed
n_trials = 15 #10 number of trials per condition
n_conditions = 5 # speed conditions: 5, 10, 15, 20 mm/s
tot_trials = n_trials * n_conditions
speed_array=numpy.zeros(tot_trials)
cnt=0
for speed in [5, 10, 15, 20, 25]: # speed conditions (mm/s)
    for j in range(n_trials):
        speed_array[cnt] = speed
        cnt=cnt+1
speed_array = speed_array.astype(int)

# randomly permute speed array
rand_indices = numpy.random.permutation(len(speed_array))
rand_speed_array = speed_array[rand_indices]

# Global parameters
vgoal = v_base

# parameter for acquistion
fps=180
dt = 1/fps
ttl_fps=fps #Match this to TTL frequency
trial_duration = 10 # second(s)
delay_time = 5 #delay bewteen trials (seconds)
elapsed_time=0

# compute the total number of samples for the stimulus
tot_samples=trial_duration*fps
end_frame=tot_samples
IMAGES_TO_GRAB=end_frame #Set to double the end_frame just ensure that all frames will be captures
IMAGES_TO_GRAB=int(round(IMAGES_TO_GRAB))

# parameters for stimulus (triggering pose estimation)
ramp_slope = 5 # mm/s^2 for getting to speed
ramp_duration = (rand_speed_array - v_base)/ramp_slope
ramp_samples = ramp_duration * fps
ramp_samples = ramp_samples.astype(int)
exit=True #Set to true to begin aquisition session

'''
Function to save videos in parallel
'''
def save_video(cam_img):
    filename='fly'+fly_num+'_Speed_'+str(v_change)+'_Trial_'+str(trial+1)
    # Camera.StopGrabbing() is called automatically by the RetrieveResult() method
    # when IMAGES_TO_GRAB images have been retrieved.
        # Create writer
    #print('C:/Users/Brandon Pratt/Desktop/Brandon/Linear Treadmill/Data/Videos/' + filename +'.mp4')
    writer=get_writer(
       save_path + filename + cam_img[0] +'.avi',  # .mp4, mkv players often support H.264, Camera1
        # test harddrive speed
       #'E:/Brandon_Test/'+ filename +'.avi',
        # use .avi (not .mp4) format because can be opened in virtualdub
        fps=fps,  # FPS is in units Hz; should be real-time...playback...set to actual
        codec='libx264',  # When used properly, this is basically
                          # "PNG for video" (i.e. lossless)
        quality=None,  # disables variable compression...0 to 10
        bitrate=None, #1000000, # set bit rate
        pixelformat='yuv420p',  # widely used
        ffmpeg_params=['-preset','ultrafast','-crf','20', '-tune', 'zerolatency'], # crf:0-51 or tune: fastdecode
        input_params=None
    )

    # write video
    for img in cam_img[1]:
        writer.append_data(img)

    #close writer
    writer.close()

    return cam_img[0] + ' saved'

while exit == True: #True

    # Recording is always specified here
    record_option = 1

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
        pylon.TlFactory.GetInstance().CreateDevice(devices[1]))

    camera2.Open()

        # camera 3
            # locate and specify camera3
    camera3 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[2]))

    camera3.Open()

        # camera 4
            # locate and specify camera3
    camera4 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[3]))

    camera4.Open()

    # camera 5
            # locate and specify camera3
    camera5 = pylon.InstantCamera(
        pylon.TlFactory.GetInstance().CreateDevice(devices[4]))

    camera5.Open()

    # Print the model name of the camera.
    print("Using device ", camera1.GetDeviceInfo().GetModelName())
    print("Using device ", camera2.GetDeviceInfo().GetModelName())
    print("Using device ", camera3.GetDeviceInfo().GetModelName())
    print("Using device ", camera4.GetDeviceInfo().GetModelName())
    print("Using device ", camera5.GetDeviceInfo().GetModelName())

   #SET PARAMETERS FOR CAMERA 1

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera1.MaxNumBuffer = 100

    #camera.Width = camera.Width.Max
    camera1.Width = 800 #592
    #camera.Height = camera.Height.Max
    camera1.Height = 400 #adjust for resizing purposes
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
        camera1.OffsetX = 0
    if genicam.IsWritable(camera1.OffsetY):
        #camera1.OffsetY = camera1.OffsetY.Min
        camera1.OffsetY = 100

    camera1.PixelFormat = "Mono8"

    camera1.AcquisitionFrameRate=camera1.ResultingFrameRate()

    print('Camera 1 resulting fps =',camera1.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 2

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera2.MaxNumBuffer = 100

    #camera.Width = camera.Width.Max
    camera2.Width = 800 #592
    #camera.Height = camera.Height.Max
    camera2.Height = 592 #560#adjust for resizing purposes
    camera2.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera2.Gain = camera2.Gain.Max
    except genicam.LogicalErrorException:
        camera2.GainRaw = camera2.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera2.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera2.OffsetX = 0
    if genicam.IsWritable(camera2.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera2.OffsetY = 0

    camera2.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera2.AcquisitionFrameRate=camera2.ResultingFrameRate()
    '''
    if fps> camera2.ResultingFrameRate():
        fps = camera2.ResultingFrameRate() #captures the actual frame rate based on the camera settings
    camera2.AcquisitionFrameRate=fps
    '''
    print('Camera 2 resulting fps =',camera2.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 3

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera3.MaxNumBuffer = 100

    #camera.Width = camera.Width.Max
    camera3.Width = 800 #592
    #camera.Height = camera.Height.Max
    camera3.Height = 400 #560#adjust for resizing purposes
    camera3.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera3.Gain = camera3.Gain.Max
    except genicam.LogicalErrorException:
        camera3.GainRaw = camera3.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera3.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera3.OffsetX = 0
    if genicam.IsWritable(camera3.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera3.OffsetY = 200

    camera3.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera3.AcquisitionFrameRate=camera3.ResultingFrameRate()
    '''
    if fps> camera2.ResultingFrameRate():
        fps = camera2.ResultingFrameRate() #captures the actual frame rate based on the camera settings
    camera2.AcquisitionFrameRate=fps
    '''
    print('Camera 3 resulting fps =',camera3.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 4

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera4.MaxNumBuffer = 100

    #camera.Width = camera.Width.Max
    camera4.Width = 800 #592
    #camera.Height = camera.Height.Max
    camera4.Height = 592 #560#adjust for resizing purposes
    camera4.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera4.Gain = camera4.Gain.Max
    except genicam.LogicalErrorException:
        camera4.GainRaw = camera4.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera4.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera4.OffsetX = 0
    if genicam.IsWritable(camera4.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera4.OffsetY = 0

    camera4.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera4.AcquisitionFrameRate=camera4.ResultingFrameRate()
    '''
    if fps> camera2.ResultingFrameRate():
        fps = camera2.ResultingFrameRate() #captures the actual frame rate based on the camera settings
    camera2.AcquisitionFrameRate=fps
    '''
    print('Camera 4 resulting fps =',camera4.ResultingFrameRate(),'Hz')

    #SET PARAMETERS FOR CAMERA 5

    # The parameter MaxNumBuffer can be used to control the count of buffers
    # allocated for grabbing. The default value of this parameter is 10.
    camera5.MaxNumBuffer = 100

    #camera.Width = camera.Width.Max
    camera5.Width = 800 #592
    #camera.Height = camera.Height.Max
    camera5.Height = 592 #560#adjust for resizing purposes
    camera5.ExposureTime = exposure_time #2 ms exposure is prefered #camera.ExposureTime.Min
    #print(camera.AcquisitionFrameRate)
    try:
        camera5.Gain = camera5.Gain.Max
    except genicam.LogicalErrorException:
        camera5.GainRaw = camera5.GainRaw.Max

        # Maximize the Image AOI.
    if genicam.IsWritable(camera5.OffsetX):
        #camera2.OffsetX = camera2.OffsetX.Min
        camera5.OffsetX = 0
    if genicam.IsWritable(camera5.OffsetY):
        #camera2.OffsetY = camera2.OffsetY.Min
        camera5.OffsetY = 0

    camera5.PixelFormat = "Mono8"

    #camera2.AcquisitionFrameRateEnable="Enabled"
    camera5.AcquisitionFrameRate=camera5.ResultingFrameRate()
    '''
    if fps> camera2.ResultingFrameRate():
        fps = camera2.ResultingFrameRate() #captures the actual frame rate based on the camera settings
    camera2.AcquisitionFrameRate=fps
    '''
    print('Camera 5 resulting fps =',camera5.ResultingFrameRate(),'Hz')

    # Perform continous aquisition in order to set initial condition
    # Configure the trigger acquistion camera1
    # Send signal out when exposure is active on Line4
    camera1.LineSelector= "Line4"
    camera1.LineMode= "Output"
    camera1.LineInverter= "False"
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
    camera2.LineInverter= "False"
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
    camera3.LineInverter= "False"
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
    camera4.LineInverter= "False"
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
    camera5.LineInverter= "False"
    camera5.LineSource="ExposureActive"

    #Set Line 4 to trigger
    camera5.LineSelector="Line3"
    camera5.LineMode="Input"
    camera5.TriggerSelector="FrameStart"
    camera5.TriggerMode="On"
    camera5.TriggerSource="Line3"
    camera5.TriggerActivation="RisingEdge"
    camera5.TriggerDelay=0

    # grabbing strategy used for viewing the images
    camera1.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    camera2.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    camera3.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    camera4.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    camera5.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    # self triggering code
    # turn on belts here to prevent the start up issues
    # set belts to the base velocity
    ch.setTargetVelocity(duty_cycle)


    for trial in range(len(rand_speed_array)):
        # specify split condition
        v_change = rand_speed_array[trial]

        #specify filename
        filename='fly'+fly_num+'_Speed_'+str(v_change)+'_Trial_'+str(trial+1)

        if trial == 0:

            # display top-down camera 5
            imageWindow_cam5 = pylon.PylonImageWindow()
            imageWindow_cam5.Create(5)

            print('Trigger to record')
            Wait = 3000

            # Perform pose estimation and determine if the imposed threshold was surpassed
            img_count = 0
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

        # Set delay object
        if trial > 0:
            # add a time delay that allows for a rest period and enough time for the video to be saved
            # d_time = time.time() - t_delay
            # print('Delay time= ',d_time, ' seconds')
            # total_delay_time = delay_time - (time.time()-t_delay)
            # delay_start = time.time()
            time.sleep(float(5)) # delay for 5 seconds
            # total_elapsed_time = time.time() - delay_start
            # print('Delayed for ',total_elapsed_time, 'seconds')

        try:
            #place images into a buffer and intialize variables
            wait_time=1500 #5000 for 200Hz
            frame_count=0 # reset frame count
            buffer=[]
            buffer_camera2=[]
            buffer_camera3=[]
            buffer_camera4=[]
            buffer_camera5=[]
            timestamp=[]
            belt_timestamp=[] # time of belt speed update
            t_start=time.time()
            vel_store=[] # store the actual velocity
            stimulus=[] #record the prescribed velocity of the belt

            while frame_count<end_frame:

                camera1.WaitForFrameTriggerReady(wait_time, pylon.TimeoutHandling_ThrowException)
                result = camera1.RetrieveResult(
                    wait_time,
                    pylon.TimeoutHandling_ThrowException)

                camera2.WaitForFrameTriggerReady(wait_time, pylon.TimeoutHandling_ThrowException)
                result_camera2=camera2.RetrieveResult(
                    wait_time,
                    pylon.TimeoutHandling_ThrowException)

                camera3.WaitForFrameTriggerReady(wait_time, pylon.TimeoutHandling_ThrowException)
                result_camera3=camera3.RetrieveResult(
                    wait_time,
                    pylon.TimeoutHandling_ThrowException)

                camera4.WaitForFrameTriggerReady(wait_time, pylon.TimeoutHandling_ThrowException)
                result_camera4=camera4.RetrieveResult(
                    wait_time,
                    pylon.TimeoutHandling_ThrowException)

                camera5.WaitForFrameTriggerReady(wait_time, pylon.TimeoutHandling_ThrowException)
                result_camera5=camera5.RetrieveResult(
                    wait_time,
                    pylon.TimeoutHandling_ThrowException)

                 #Image grabbed successfully?
                if result.GrabSucceeded() and result_camera2.GrabSucceeded() and result_camera3.GrabSucceeded() and result_camera4.GrabSucceeded() and result_camera5.GrabSucceeded():

                    # record time
                    curr_time=float(time.time())
                    timestamp.append(curr_time)

                    #record stimulus velocity (right)
                    stimulus.append(ch.getVelocity())

                    # append buffer of camera 1
                    curr_frame=result.Array
                    buffer.append(curr_frame)

                    # append buffer of camera 2
                    curr_frame_cam2=result_camera2.Array
                    buffer_camera2.append(curr_frame_cam2)

                    # append buffer of camera 3
                    curr_frame_cam3=result_camera3.Array
                    buffer_camera3.append(curr_frame_cam3)

                    # append buffer of camera 4
                    curr_frame_cam4=result_camera4.Array
                    buffer_camera4.append(curr_frame_cam4)

                    # append buffer of camera 5
                    curr_frame_cam5=result_camera5.Array
                    buffer_camera5.append(curr_frame_cam5)

                    # check and update belt speed
                    if frame_count <= ramp_samples[trial]:
                        vgoal = (ramp_slope*(dt * frame_count)) + v_base

                    else:
                        vgoal = v_change

                    # Update frame counter
                    frame_count=frame_count+1

                else:
                    print("Error: ", result.GetErrorCode(), result.GetErrorDescription())
                    print("Error: ", result_camera2.GetErrorCode(), result_camera2.GetErrorDescription())
                    print("Error: ", result_camera3.GetErrorCode(), result_camera3.GetErrorDescription())
                    print("Error: ", result_camera4.GetErrorCode(), result_camera4.GetErrorDescription())
                    print("Error: ", result_camera5.GetErrorCode(), result_camera5.GetErrorDescription())
                result.Release()
                result_camera2.Release()
                result_camera3.Release()
                result_camera4.Release()
                result_camera5.Release()

            # reset driving speed to the basline speed
            vgoal = v_base

            # Isolate only the belt updates (speed and time) during the recording session
            belt_time = belt_timestamp
            belt_speed = vel_store

            # treadmill belt speed update time
            belt_timestamps = numpy.array(belt_time)
            numpy.savetxt(save_path + filename+'_Belt_TimeStamps.txt', belt_timestamps, delimiter=',')

            # save converted numpy array as text file
            belt_stimulus = numpy.array(belt_speed)
            numpy.savetxt(save_path + filename+'_Belt_Speed.txt', belt_stimulus, delimiter=',')
            numpy.savetxt(save_path + filename+'_Prescribed_Belt_Speed.txt', numpy.array(stimulus), delimiter=',')
            print('Saving Videos')

            #compute the time for delaying between stimuli bouts
            t_delay=time.time()

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
            print('Finsihed Saving')

        except genicam.GenericException as e:
            # Error handling.
            print("An exception occurred.", e.GetDescription())
            #exitCode = 1

    # close phidget channel
    ch.close()

    # camera has to be closed manually
    camera1.Close()
    camera2.Close()
    camera3.Close()
    camera4.Close()
    camera5.Close()

    exit=False
