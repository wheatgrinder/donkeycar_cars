#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it.

Usage:
    manage.py (drive) [--model=<model>] [--js] [--chaos]
    manage.py (train) [--tub=<tub1,tub2,..tubn>]  (--model=<model>) [--base_model=<base_model>] [--no_cache]

Options:
    -h --help        Show this screen.
    --tub TUBPATHS   List of paths to tubs. Comma separated. Use quotes to use wildcards. ie "~/tubs/*"
    --js             Use physical joystick.
    --chaos          Add periodic random steering when manually driving
"""
import os
from docopt import docopt

import donkeycar as dk
from donkeycar.parts.camera import PiCamera
from donkeycar.parts.transform import Lambda
#from donkeycar.parts.keras import KerasLinear
from donkeycar.parts.keras import KerasCategorical
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.parts.datastore import TubGroup, TubWriter
from donkeycar.parts.clock import Timestamp
# use custom controller for inputs.. 
#from controller import LocalWebController, JoystickController 

#custom donkeypet parts
from donkeycar.parts.controller_with_pan_and_tilt import LocalWebController, JoystickController
from donkeycar.parts.pan_tilt_cam import PWMPanning, PWMTilting, PWMWagging
from donkeycar.parts.PCA9685_LED import rgb_led, turn_signal, status_indicator
from donkeycar.parts.cv import ImgResize
from donkeycar.parts.cv import ImgPutText

from donkeycar.parts.perfmon import driveLoopTime
from donkeycar.parts.perfmon import coreTemp
from donkeycar.parts.perfmon import throttled

from donkeycar.parts.emotes import target_status


#NCS PARTS
#from donkeycar.parts.ncs import googlenet
#from donkeycar.parts.ncs import inception
#from donkeycar.parts.ncs import tinyyolo

#face detection part
from donkeycar.parts.face import detect

from donkeycar.parts.remote_donkey import send_to_donkey,rcv_from_donkey

def drive(cfg, model_path=None, use_joystick=False, use_chaos=False):
    
     
    """
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    """

    V = dk.vehicle.Vehicle()
    
    center_led = rgb_led(red_channel = 9, green_channel = 10, blue_channel = 11)  
    status_led = status_indicator(status_led = center_led)
    V.add(center_led, outputs=['none'])
    V.add(status_led, inputs=['user/mode', 'recording'])

    #add pi_perfchecker
    #loop_time = driveLoopTime()
    #loop_time.console=True
    #core_temp = coreTemp()
    #V.add(core_temp)
    
    #V.add(loop_time,inputs = ['timein'], outputs = ['timein','displaytext'])
    
    #throtled_status = throttled()
    #V.add(throtled_status, outputs=['displaytext'],threaded=True) 


    loop_time = driveLoopTime()
    V.add(loop_time,inputs = ['timein'], outputs = ['timein','displaytext'])
    loop_time_display = ImgPutText(org=(100,20))



    clock = Timestamp()
    V.add(clock, outputs=['timestamp'])

    cam = PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    #cam = PiCamera(resolution=(960,1280))
   
    V.add(cam, outputs=['cam/image_array'], threaded=True)

    #ncs_ty = tinyyolo(basedir = cfg.MODELS_PATH, draw_on_img = True, probability_threshold = 0.15,debug=False)
    #V.add(ncs_ty, inputs=['cam/image_array'],outputs=['cam/image_array','ncs/found_objs'],threaded=True)
    
    V.add(loop_time_display,inputs=['cam/image_array','displaytext'], outputs=['cam/image_array'])


    #img_resize = ImgResize()
    #V.add(img_resize,inputs=['cam/image_array'],outputs=['cam/image_array'])
    #ncs_gn = googlenet(basedir=cfg.MODELS_PATH, debug=True)
    #V.add(ncs_gn, inputs=['cam/image_array'],outputs=['ncs/image_array', 'classificaiton'],threaded=True)
     
    #ncs_inception = inception(basedir=cfg.MODELS_PATH, probability_threshold=0.01, debug=True)
    #V.add(ncs_inception, inputs=['cam/image_array'],outputs=['cam/image_array', 'classificaiton'],threaded=True)
    
 
    
    #face_detect = detect(basedir=cfg.MODELS_PATH, debug=True)
   # V.add(face_detect, inputs=['cam/image_array'],outputs=['face/image_array', 'face_center'],threaded=True)
 

    
    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        ctr = JoystickController(max_throttle=cfg.JOYSTICK_MAX_THROTTLE,
                                steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                                auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE,
                                throttle_axis='rz', #throttle_axis='y',
                                steering_axis='x',
                                panning_axis='rx',
                                tilting_axis='ry')
        """ 
        when running from JS, let us display the nsc images on the web page
        """
        """
        ctr_webview = LocalWebController()
        V.add(ctr_webview,          
                inputs=['cam/image_array'], 
                #inputs=['cam/image_array'], 
                outputs=['user/angleX', 'user/throttleX', 'user/modeX', 'recordingX'], 
                #outputs=['outargs'], 
                threaded=True)
        
                
    else:
        # This web controller will create a web server that is capable
        # of managing steering, throttle, and modes, and more.
        ctr = LocalWebController(use_chaos=use_chaos)
            """




    V.add(ctr,
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording','user/pan','user/tilt'],
          #outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)
    
    ctr_webview = LocalWebController()
    V.add(ctr_webview,          
            inputs=['cam/image_array'], 
            #inputs=['cam/image_array'], 
            outputs=['user/angleX', 'user/throttleX', 'user/modeX', 'recordingX'], 
            #outputs=['outargs'], 
            threaded=True)

    
    # See if we should even run the pilot module.
    # This is only needed because the part run_condition only accepts boolean
    def pilot_condition(mode):
        if mode == 'user':
            return False
        else:
            return True

    pilot_condition_part = Lambda(pilot_condition)
    V.add(pilot_condition_part,
          inputs=['user/mode'],
          outputs=['run_pilot'])

    # Run the pilot if the mode is not user.
    #kl = KerasLinear()
    kl = KerasCategorical()
    if model_path:
        kl.load(model_path)

    V.add(kl,
          inputs=['cam/image_array'],
          outputs=['pilot/angle', 'pilot/throttle'],
          run_condition='run_pilot')



    # Choose what inputs should change the car.
    def drive_mode(mode,
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle,
                   user_pan,user_tilt,
                   face_pan,face_tilt):
                   
        if mode == 'user':
            return user_angle, user_throttle,user_pan,user_tilt
        elif mode == 'face':
            return user_angle, user_throttle,face_pan,face_tilt
        elif mode == 'local_angle':
            return pilot_angle, user_throttle,face_pan,face_tilt

        else:
            return pilot_angle, pilot_throttle,face_pan,face_tilt

    drive_mode_part = Lambda(drive_mode)
    V.add(drive_mode_part,
          inputs=['user/mode',
                  'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle',
                  'user/pan','user/tilt',
                  'face/pan','face/tilt'],
          outputs=['angle', 'throttle','pan','tilt'])

    steering_controller = PCA9685(cfg.STEERING_CHANNEL)
    steering = PWMSteering(controller=steering_controller,
                           left_pulse=cfg.STEERING_LEFT_PWM,
                           right_pulse=cfg.STEERING_RIGHT_PWM)

    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL)
    throttle = PWMThrottle(controller=throttle_controller,
                           max_pulse=cfg.THROTTLE_FORWARD_PWM,
                           zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                           min_pulse=cfg.THROTTLE_REVERSE_PWM)

    panning_controller = PCA9685(cfg.PAN_CHANNEL)
    panning = PWMPanning(controller=panning_controller,
                                    left_pulse=cfg.PAN_LEFT_PWM, 
                                    zero_pulse=cfg.PAN_CENTER_PWM,
                                    right_pulse=cfg.PAN_RIGHT_PWM)

    tilting_controller = PCA9685(cfg.TILT_CHANNEL)
    tilting = PWMTilting(controller=tilting_controller,
                                    max_pulse=cfg.TILT_UP_PWM,
                                    zero_pulse=cfg.TILT_DRIVING_PWM, 
                                    min_pulse=cfg.TILT_DOWN_PWM)   

    wagging_controller = PCA9685(cfg.TAIL_CHANNEL)
    wagging = PWMWagging(controller=wagging_controller,
                                    max_pulse=cfg.TAIL_UP_PWM,
                                    zero_pulse=cfg.TAIL_CENTER_PWM, 
                                    min_pulse=cfg.TAIL_DOWN_PWM)

   


    V.add(steering, inputs=['angle'])
    V.add(throttle, inputs=['throttle'])


    #looping pan and tilt out to donkeyface, then back into this donkeypet
    donkeyPet_sender = send_to_donkey(send_to_address='10.11.44.20')
    V.add(donkeyPet_sender,inputs=['pan','tilt','user/mode','recording','timestamp'],outputs = ['donkeyface_output'])
    
    donkeyPet_receiver = rcv_from_donkey(listen_to_port=10500, debug=False)
    V.add(donkeyPet_receiver,inputs=['rcv_in'],outputs = ['face/pan','face/tilt','face/irq'])

    feels_on_target = target_status(debug=True)
    V.add(feels_on_target, inputs=['face/irq','tail','user/mode'],outputs=['tail'])



    V.add(panning, inputs=['face/pan'])
    V.add(tilting, inputs=['face/tilt'])
    V.add(wagging, inputs=["tail"])
   
    # add tub to save data
    #pan and tilt setup
    
   
 
    

    
    tub_inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode','timestamp']
    tub_types = ['image_array', 'float', 'float', 'str', 'str']
    
    
 

    #non pan and tilt test
    #tub_inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode', 'timestamp','test_var']
    #tub_types = ['image_array', 'float', 'float', 'str', 'str','str']



    # multiple tubs
    # th = TubHandler(path=cfg.DATA_PATH)
    # tub = th.new_tub_writer(inputs=inputs, types=types)

    # single tub
    tub = TubWriter(path=cfg.TUB_PATH, inputs=tub_inputs, types=tub_types)
    V.add(tub, inputs=tub_inputs, run_condition='recording')

    # run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)


def train(cfg, tub_names, new_model_path, base_model_path=None ):
    """
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    """
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'user/throttle']
    def train_record_transform(record):
        """ convert categorical steering to linear and apply image augmentations """
        record['user/angle'] = dk.util.data.linear_bin(record['user/angle'])
        # TODO add augmentation that doesn't use opencv
        return record

    def val_record_transform(record):
        """ convert categorical steering to linear """
        record['user/angle'] = dk.util.data.linear_bin(record['user/angle'])
        return record

    new_model_path = os.path.expanduser(new_model_path)

    kl = KerasCategorical()
    if base_model_path is not None:
        base_model_path = os.path.expanduser(base_model_path)
        kl.load(base_model_path)

    print('tub_names', tub_names)
    if not tub_names:
        tub_names = os.path.join(cfg.DATA_PATH, '*')
    tubgroup = TubGroup(tub_names)
    train_gen, val_gen = tubgroup.get_train_val_gen(X_keys, y_keys,
                                                    train_record_transform=train_record_transform,
                                                    val_record_transform=val_record_transform,
                                                    batch_size=cfg.BATCH_SIZE,
                                                    train_frac=cfg.TRAIN_TEST_SPLIT)

    total_records = len(tubgroup.df)
    total_train = int(total_records * cfg.TRAIN_TEST_SPLIT)
    total_val = total_records - total_train
    print('train: %d, validation: %d' % (total_train, total_val))
    steps_per_epoch = total_train // cfg.BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)

    kl.train(train_gen,
             val_gen,
             saved_model_path=new_model_path,
             steps=steps_per_epoch,
             train_split=cfg.TRAIN_TEST_SPLIT)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg, model_path = args['--model'], use_joystick=args['--js'], use_chaos=args['--chaos'])

    elif args['train']:
        tub = args['--tub']
        new_model_path = args['--model']
        base_model_path = args['--base_model']
        cache = not args['--no_cache']
        train(cfg, tub, new_model_path, base_model_path)


