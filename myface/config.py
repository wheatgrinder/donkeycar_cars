
"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/mycar/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#CAMERA
#CAMERA_RESOLUTION = (160, 120) #(height, width)
#CAMERA_RESOLUTION = (120, 160) #(height, width)
CAMERA_RESOLUTION = (448,448) #(height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 285  
#300
STEERING_RIGHT_PWM = 475 
#460 
#THROTTLE
THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM =480 
#480 
THROTTLE_STOPPED_PWM = 410 
THROTTLE_REVERSE_PWM = 310 


# neck 
# neck tilt
# channel = 3
TILT_CHANNEL = 3
TILT_UP_PWM = 150
TILT_DOWN_PWM = 525
TILT_CENTER_PWM = 260
TILT_DRIVING_PWM = 320

# tilt down max = 525
# tilt down looking straight down at ground 525 clears platrom etc..
# tilt level looking direct forward  260


# neck pan
# channel = 2
PAN_CHANNEL = 2
PAN_RIGHT_PWM = 85
PAN_LEFT_PWM = 600
PAN_CENTER_PWM = 325
# pan max right 60
# pan max right 100 clears all
# pan max left 520..clears all 
# pan max left 570
# center forward 295
#  

TAIL_CHANNEL = 15
TAIL_UP_PWM = 300
TAIL_DOWN_PWM = 100
TAIL_CENTER_PWM = 200



#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8


#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = False
JOYSTICK_MAX_THROTTLE = 1.0
JOYSTICK_STEERING_SCALE = 1.0
AUTO_RECORD_ON_THROTTLE = False



TUB_PATH = os.path.join(CAR_PATH, 'tub') # if using a single tub
#ROPE.DONKEYCAR.COM
ROPE_TOKEN='cb7ea3fb89e704fe4b43359b476950228903f74e' 
ROPE_BOT_NAME='None'

