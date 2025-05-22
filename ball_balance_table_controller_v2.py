from __future__ import division
from threading import Thread
import evdev
import time
from evdev import ecodes
import datetime
import numpy as np
from controller_not_started_error import ControllerNotStartedError
from servo_controller import HS5645MGServoController


class BallBalanceTableControllerv2(object):
    # TOUCH_CONTROLLER_NAME = "eGalax Inc. USB TouchController"
    TOUCH_CONTROLLER_NAME = "eGalax Inc."

    def __init__(self, logger=None,x_filter=3,y_filter=3,d_filter = 3,i_filter=30,integral_max=136,velocity_filter = 500,K_P=(0.2, 0.206),K_I= (0.0, 0.0),K_D= (0.125, 0.1256),
                printResults=False,loop_fix_time = 5000,filter_type = 'linear'):

        self._servo_controller = HS5645MGServoController()

        # Variables for touch panel
        self.X_LENGTH = 344  # In mm old:248
        self.Y_LENGTH = 272  # In mm old:282
        self.THEORETICAL_ORIGIN_POSITION = (1023.5, 1023.5)
        self.THEORETICAL_END_POSITION = (2047, 2047)
        self._touch_controller_dev = None
        # self._TOUCH_CONTROLLER_NAME = "eGalax Inc."
        self._last_ball_position_reading = (0, 0)  # x, y
        self._touch_events_listening_thread = None
        self._keep_listening_touch_events = False
        self._started = False

        # Getting variables to inside of class
        self._logger = logger
        self._x_filter = x_filter
        self._i_filter = i_filter
        self._y_filter = y_filter
        self._d_filter = d_filter
        self._velocity_filter = velocity_filter
        self._loop_fix_time = loop_fix_time
        self._K_P = K_P
        self._K_I = K_I
        self._K_D = K_D
        self._printResults = printResults
        self._filter_type = filter_type
        self._integral_max = integral_max

        self.b_pos = (0, 0)  # Ball position in mm
        self.b_pos_mapped = (0, 0)  # Mapped ball position (30 - 150)
        self.old_b_pos_mapped = (0,0)
        self.p_error = (0.0, 0.0)
        self.current_error = (0.0, 0.0)
        self.last_error = (0.0, 0.0)
        self.delta_error = (0.0, 0.0)
        self.cycle_counter = 0
        self.DESIRED_POSITION = (0,0)
        self.REF_POSITION = self.DESIRED_POSITION
        self.last_time = datetime.datetime.now()
        self.current_time = datetime.datetime.now()
        self._write_timer = 0
        self._touch_controller_dev = None


        self._practical_origin_position = self.THEORETICAL_ORIGIN_POSITION
        self._practical_end_position = self.THEORETICAL_END_POSITION
        self.dt = 0.0
        self.p = (0.0, 0.0)
        self.i = (0.0, 0.0)
        self.d = (0.0, 0.0)
        self.pid = (0.0, 0.0)
        self.x_filter_terms = np.array([(0)]*self._x_filter,dtype='f')
        self.y_filter_terms = np.array([(0)]*self._y_filter,dtype='f')
        self.i_terms = np.array([(0, 0)]*self._i_filter,dtype='f')
        self.d_terms = np.array([(0, 0)]*self._d_filter,dtype='f')
        self.i_error = np.array([0,0],dtype='f')
        self.velocity_terms = np.array([(0, 0)]*self._velocity_filter,dtype='f')

        self.current_offset_x = 0
        self.current_offset_y = 0
        self.ball_contact = False
        self._gauss_array = ([0])*self._x_filter
        self.current_servo_positions = (90,90)
        self.last_position = (0,0)
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity  = (1,1)
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration  = (1,1)
        self.y_filter_counter = 0
        self.i_counter = 0
        self.d_counter = 0
        self.x_filter_counter = 0
        self.vel_counter = 0
        self._last_ball_position_reading = (0, 0)  # x, y
        self._running_thread = False
        self._stop_flag = False
        self._test_number = 0
        self._test_count = 0
        self._wait_flag = False
        self._first_cycle_completed = False
        self._initialized = False
        self._armed = False
        self._first_touch = False

    def arm_system(self,init_wiringpi=True):
            if self._logger is not None:
               self._logger.error("Starting Ball Balance Table communication.")

            if self._started:
                if self._logger is not None:
                    self._logger.error("Ball Balance Table communication already started.")
                return

            self._servo_controller.start()
            print("sdfghjk")
            self.start_listening_touch_events()
            time.sleep(1)

            if self._logger is not None:
                self._logger.debug("Ball Balance Table communication started.")


            self._armed = True
            self._initialized = True

    def start_control(self):

        self.x_filter_terms = np.array([(0)]*self._x_filter,dtype='f')
        self.y_filter_terms = np.array([(0)]*self._y_filter,dtype='f')
        self.d_terms = np.array([(0, 0)]*self._d_filter,dtype='f')
        self.velocity_terms = np.array([(0, 0)]*self._velocity_filter,dtype='f')
        self._started = True
        self._running_thread = True

	
    def stop_control(self):
        self._started = False
        self._running_thread = False
        if self._logger is not None:
            self._logger.debug("Ball Balance Table Control stopped.")
    		
    def get_ball_position(self):
        return self.b_pos_mapped
    def get_init_status(self):
        return self._initialized
    def get_start_status(self):
        return self._started

			
    def start(self, init_wiringpi=True):
        if self._logger is not None:
            self._logger.error("Starting Ball Balance Table communication.")
        if self._started:
            if self._logger is not None:
                self._logger.error("Ball Balance Table communication already started.")
            return
        self._servo_controller.start()
        self.start_listening_touch_events()
        time.sleep(1)
        self._started = True
        self._running_thread = False
        if self._logger is not None:
            self._logger.debug("Ball Balance Table communication started.")
        self._initialized = True
    def translate(self, value, leftMin, leftMax, rightMin, rightMax): # Setting value from leftMin-leftMax to rightMin-rightMax.
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)
        returnVal = rightMin + (valueScaled * rightSpan)
        # If returnVal is out of boundries, saturate returnVa
        if rightMin < rightMax:
            if returnVal > rightMax:
                returnVal = rightMax
            elif returnVal < rightMin:
                returnVal = rightMin
        else:
            if returnVal < rightMax:
                returnVal = rightMax
            elif returnVal > rightMin:
                returnVal = rightMin
       # Convert the 0-1 range into a value in the right range.
        return returnVal
    def _touch_events_listening_thread_func(self):
        if self._logger is not None:
            self._logger.debug("Starting listening to touch events.")
        while self._keep_listening_touch_events:
            # This while will run on a different thread.
            try:
                event= None
                event = self._touch_controller_dev.read_one()
                if event is not None:
                    if event.type == ecodes.EV_ABS:
                        if event.code == ecodes.ABS_X:
                            self._last_ball_position_reading = (event.value, self._last_ball_position_reading[1])
                        if event.code == ecodes.ABS_Y:
                            self._last_ball_position_reading = (self._last_ball_position_reading[0], event.value)
                    if event.type == ecodes.EV_KEY:
                        if event.value == 1:
                            self.ball_contact = True
                        else:
                            self.ball_contact = False

                else:
                    time.sleep(0.001) # If there is no change from last loop, sleep 1ms
            except OSError:
                pass
        if self._logger is not None:
            self._logger.debug("Started listening to touch events.")
    def start_listening_touch_events(self):
        self._touch_controller_dev = self.find_touch_controller_dev()
        self._keep_listening_touch_events = True
        # Start touch panel thread
        self._touch_events_listening_thread = Thread(target=self._touch_events_listening_thread_func)
        self._touch_events_listening_thread.start()
    @staticmethod
    def find_touch_controller_dev():
        touch_controller_device = None
        # Initialize touch panel device
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            if BallBalanceTableControllerv2.TOUCH_CONTROLLER_NAME in device.name:
                touch_controller_device = device
        return touch_controller_device
    
    def get_ball_position_in_mm(self): # To read position from outside of class in mm format
        if not self._started:
            raise ControllerNotStartedError
        position_x = self.translate(self._last_ball_position_reading[0], 157, 1976, -172, 172)# 195 1973
        position_y = self.translate(self._last_ball_position_reading[1], 121, 1920, -136, 136)# 123 1916
        return (position_x, position_y)
    
    def ball_position_raw():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                x_pos=event.value
            elif event.code == ecodes.ABS_Y:
                y_pos=event.value
        return (x_pos, y_pos)

