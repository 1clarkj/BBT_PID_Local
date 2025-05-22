from __future__ import division

import sys
import time
import random
import pigpio
import threading


class HS5645MGServoController(object):
    COMM_FREQUENCY = 50
    NEUTRAL_POSITION_DUTY_CYCLE = 1500
    PWM_CHANNEL_1_GPIO_PIN = 12
    PWM_CHANNEL_2_GPIO_PIN = 13


    MIN_WIDTH = 750
    MAX_WIDTH = 2250
    def __init__(self, signal_pin_x=PWM_CHANNEL_1_GPIO_PIN, signal_pin_y = PWM_CHANNEL_2_GPIO_PIN, type = 'BBT', logger=None):
        self._type = type
        if self._type == 'BBT':
            self._signal_pin_x = signal_pin_x
            self._signal_pin_y = signal_pin_y
        else:
            self._signal_pin_x = signal_pin_x

        self._logger = logger

        self._communication_lock = threading.Lock()
        self._pwm = None
        self._duty_cycle = None

        self._started = False
        self.pi = pigpio.pi()

    def start(self):

        if self._logger is not None:
            self._logger.debug("Servo communication started.")

        self._started = True
    def translate(self,value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)
        returnVal = rightMin + (valueScaled * rightSpan)
        if rightMin<rightMax:
            if returnVal>rightMax:
                returnVal = rightMax
            elif returnVal<rightMin:
                returnVal = rightMin
        else:
            if returnVal<rightMax:
                returnVal = rightMax
            elif returnVal>rightMin:
                returnVal = rightMin

        # Convert the 0-1 range into a value in the right range.
        return returnVal

    def get_gpio_value(self,gpio_number):
        val = self.pi.read(gpio_number)
        return val

    def set_gpio_value(self,gpio_number,val):
        if val==1:
            self.pi.write(gpio_number,1)
        else:
            self.pi.write(gpio_number,0)
    def set_duty_cycle_bb(self,duty_cycle):
        self.pi.set_servo_pulsewidth(self._signal_pin_x, int(duty_cycle))

    def set_duty_cycle_bbt(self, duty_cycle):
        self.pi.set_servo_pulsewidth(self._signal_pin_x, int(duty_cycle[0]))
        self.pi.set_servo_pulsewidth(self._signal_pin_y, int(duty_cycle[1]))

    def set_degrees_bb(self,degree):
        if self._logger is not None:
            self._logger.debug("Setting servo angle in degrees.")

        with self._communication_lock:
            duty_cycle_x = self.translate(degree,90,-90,750,2250)

            # print(int(duty_cycle))
            self.set_duty_cycle_bb(duty_cycle_x)

    def set_degrees_bbt(self, degrees):
        if self._logger is not None:
            self._logger.debug("Setting servo angle in degrees.")

        with self._communication_lock:
            duty_cycle_x = self.translate(degrees[0],90,-90,750,2250)
            duty_cycle_y = self.translate(degrees[1],90,-90,750,2250)

            # print(int(duty_cycle))
            self.set_duty_cycle_bbt((duty_cycle_x,duty_cycle_y))
        # Map 30-150 degrees to 5-10 duty cycle

    def close(self):

        if self.pi is not None:
            self.pi.set_servo_pulsewidth(self._signal_pin_x,0)
            self.pi.set_servo_pulsewidth(self._signal_pin_y,0)
            self.pi.stop()
            if self._logger is not None:
                self._logger.debug("Servo communication closed.")
