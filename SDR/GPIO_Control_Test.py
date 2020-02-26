#-------------------------------------------------------------------------------
# Name:       Switch Test
# Purpose:
#
# Author:      camil
#
# Created:     26/02/2020
# Copyright:   (c) camil 2020
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import RPi.GPIO as GPIO
import time

for i in range(1,3):

    GPIO.setmode(GPIO.BOARD)
    PinV1 = 'GPIO17'
    PinV2 = 'GPIO27'
    PinV3 = 'GPIO22'

    GPIO.setup([PinA,PinB,PinC], GPIO.OUT)

    if config == 1: # RFC--> RF1
        GPIO.output(PinV1, GPIO.HIGH)
        GPIO.output(PinV2, GPIO.LOW)
        GPIO.output(PinV3, GPIO.LOW)
    if config == 2: # RFC--> RF2
        GPIO.output(PinV1, GPIO.LOW)
        GPIO.output(PinV2, GPIO.HIGH)
        GPIO.output(PinV3, GPIO.LOW)
    if config ==3: # RFC--> RF3
        GPIO.output(PinV1, GPIO.LOW)
        GPIO.output(PinV2, GPIO.LOW)
        GPIO.output(PinV3, GPIO.HIGH)
    time.sleep(5)






