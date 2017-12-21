#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it. 

Usage:
    manage.py (drive) [--model=<model>] [--js]
    manage.py (train) [--tub=<tub1,tub2,..tubn>]  (--model=<model>) [--no_cache]

Options:
    -h --help        Show this screen.
    --tub TUBPATHS   List of paths to tubs. Comma separated. Use quotes to use wildcards. ie "~/tubs/*"
    --js             Use physical joystick.
"""
import os,time
from docopt import docopt
 
import donkeycar as dk


#import parts
from donkeycar.parts.camera import PiCamera
#from donkeycar.parts.cv import ImgGreyscale
#from donkeycar.parts.cv import ImgCanny
from donkeycar.parts.cv import ImgCrop
from donkeycar.parts.cv import ImgDrawLine
from donkeycar.parts.cv import ImgPutInfo
from donkeycar.parts.cv import ImgPutText
from donkeycar.parts.cv import ImgResize
from donkeycar.parts.transform import Lambda
from donkeycar.parts.keras import KerasCroppedCategorical
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.datastore import TubHandler, TubGroup
from donkeycar.parts.controller import LocalWebController, JoystickController

#NCS PARTS
from donkeycar.parts.ncs import googlenet
from donkeycar.parts.ncs import tinyyolo


from donkeycar.parts.perfmon import driveLoopTime
from donkeycar.parts.perfmon import coreTemp
from donkeycar.parts.perfmon import throttled

def drive(cfg, model_path=None, use_joystick=False):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    ''' 
    
    #Initialize car
    V = dk.vehicle.Vehicle()
    #add pi_perfchecker
    loop_time = driveLoopTime()
    loop_time.console=False 
    #core_temp = coreTemp()
    #throtled_status = throttled()
    V.add(loop_time,inputs = ['timein'], outputs = ['timein','displaytext'])
    #V.add(core_temp)
    #V.add(throtled_status)
 
    cam = PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    V.add(cam, outputs=['cam/image_array'], threaded=True)
 
   
    #ncs_gn = googlenet(basedir=cfg.MODELS_PATH)
    #V.add(ncs_gn, inputs=['cam/image_array'],outputs=['classificaiton'],threaded=True)
    
    ncs_ty = tinyyolo(basedir=cfg.MODELS_PATH)
    V.add(ncs_ty, inputs=['cam/image_array'],outputs=['ncs/image_array'],threaded=True)
    
    loop_time_display = ImgPutText()
    
    V.add(loop_time_display,inputs=['ncs/image_array','displaytext'], outputs=['ncs/image_array'])
    #classify = ImgPutText()
    #V.add(classify,inputs=['cam/image_array','classificaiton'],outputs=['ncs/image_array'])
    
    # draw a line showing where training input is cropped
    #l1 = ImgDrawLine()
    #l1.start = (0,39)
    #l1.end = (160,39)
    #l1.color = (0,255,0)
    #l1.width=10
    #V.add(l1, inputs=['ncs/image_array'],outputs=['ncs/image_array'])

    
 
    #driverInfo = ImgPutInfo()
    #throttleText.text='SPD:'
    #V.add(driverInfo, inputs=['cam/image_array','throttle', 'angle'],outputs=['cam/image_array'])
    #greyScale = ImgGreyscale()
    #imgCanny = ImgCanny()

    # moving croping to network input layer rather than image.. 
    # Allows me to see and process the entire image but only conside
    # the bottom third for stearing input to the network..
    
    #imgCrop = ImgCrop(top=40,bottom=0,left=0,right=0)
    #V.add(imgCrop,  inputs=['cam/image_array'],outputs=['cam/image_array'])


    #V.add(greyScale, inputs=['cam/image_array'],outputs=['cam/image_array'])
    
    

    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        #modify max_throttle closer to 1.0 to have more power
        #modify steering_scale lower than 1.0 to have less responsive steering
        ctr = JoystickController(max_throttle=cfg.JOYSTICK_MAX_THROTTLE,
                                 steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                                 auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE)
    else:        
        #This web controller will create a web server that is capable
        #of managing steering, throttle, and modes, and more.
        ctr = LocalWebController()
        #ctr.auto_record_on_throttle = False
    
    V.add(ctr,          
          inputs=['ncs/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)
    
    #See if we should even run the pilot module. 
    #This is only needed because the part run_condition only accepts boolean
    def pilot_condition(mode):
        if mode == 'user':
            return False
        else:
            return True
        
    pilot_condition_part = Lambda(pilot_condition)
    V.add(pilot_condition_part, inputs=['user/mode'], outputs=['run_pilot'])
    

    #Run the pilot if the mode is not user.
    kl = KerasCroppedCategorical()
    if model_path:
        kl.load(model_path)
    
    kResizeImg= ImgResize()
    V.add(kResizeImg, inputs=['cam/image_array'], outputs=['resized/img_array'])

    #V.add(kl, inputs=['cam/image_array'], 
    V.add(kl, inputs=['resized/img_array'],
          outputs=['pilot/angle', 'pilot/throttle'],
          run_condition='run_pilot')
    
    
    #Choose what inputs should change the car.
    def drive_mode(mode, 
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle):
        if mode == 'user': 
            return user_angle, user_throttle
        
        elif mode == 'local_angle':
            #return pilot_angle, user_throttle
            return pilot_angle, 0.30
        else: 
            #overite throttle
            #return pilot_angle, pilot_throttle
            #return pilot_angle, 0.80
            return pilot_angle, pilot_throttle 
        
        
    drive_mode_part = Lambda(drive_mode)
    V.add(drive_mode_part, 
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'], 
          outputs=['angle', 'throttle'])
    
    
    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, 
                                    right_pulse=cfg.STEERING_RIGHT_PWM)


    
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    #throttleText.text = throttle
    
    V.add(steering, inputs=['angle'])
    V.add(throttle, inputs=['throttle'])
    
    #add tub to save data
    inputs=['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
    types=['image_array', 'float', 'float',  'str']
    
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)
    V.add(tub, inputs=inputs, run_condition='recording')
    
    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)
    
    print("You can now go to <your pi ip address>:8887 to drive your car.")


def train(cfg, tub_names, model_name):
    '''
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    '''
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'user/throttle']

    def rt(record):
        record['user/angle'] = dk.utils.linear_bin(record['user/angle'])
        return record

    kl = KerasCroppedCategorical()
    print('tub_names', tub_names)
    if not tub_names:
        tub_names = os.path.join(cfg.DATA_PATH, '*')
    tubgroup = TubGroup(tub_names)
    train_gen, val_gen = tubgroup.get_train_val_gen(X_keys, y_keys, record_transform=rt,
                                                    batch_size=cfg.BATCH_SIZE,
                                                    train_frac=cfg.TRAIN_TEST_SPLIT)

    model_path = os.path.expanduser(model_name)

    total_records = len(tubgroup.df)
    total_train = int(total_records * cfg.TRAIN_TEST_SPLIT)
    total_val = total_records - total_train
    print('train: %d, validation: %d' % (total_train, total_val))
    steps_per_epoch = total_train // cfg.BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)

    kl.train(train_gen,
             val_gen,
             saved_model_path=model_path,
             steps=steps_per_epoch,
             train_split=cfg.TRAIN_TEST_SPLIT)





if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path = args['--model'], use_joystick=args['--js'])

    elif args['train']:
        tub = args['--tub']
        model = args['--model']
        cache = not args['--no_cache']
        train(cfg, tub, model)





