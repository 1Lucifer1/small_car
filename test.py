#!/usr/bin/python3
# -*- encoding : utf-8 -*-
import time
import sys
import RPi.GPIO as GPIO

CENTER = 7
FRONT = 8
RIGHT = 9
LEFT = 10
BACK = 11
RA = 21
RB = 20
LA = 13
LB = 12
CONTROLR = 26
CONTROLL = 6
speed_rate = 10
buzzer = 4

CS = 5
CLOCK = 25
ADDRESS = 24
DATAOUT = 23
stop_signal = 0
pingroup = (LEFT,RIGHT,CENTER,FRONT,BACK)

GPIO.setmode(GPIO.BCM)
GPIO.setup(pingroup,GPIO.IN,GPIO.PUD_UP)
message = {CENTER: 'CENTER', LEFT: 'LEFT',RIGHT: 'RIGHT',FRONT: 'FRONT',BACK: 'BACK'}

def pin_catched(channel):
    sys.exit(0)
    print (message[channel])
    print(stop)

Trig = 22
Echo = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(Trig,GPIO.OUT)
GPIO.setup(Echo,GPIO.IN)

def ranging():
    GPIO.output(Trig,GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(Trig,GPIO.LOW)
    while GPIO.input(Echo)==GPIO.LOW:
        pass
    t1 = time.time()
    while GPIO.input(Echo)==GPIO.HIGH:
        pass
    t2 = time.time()
    return (t2 - t1)*34000/2

GPIO.setmode(GPIO.BCM)
GPIO.setup((CS, CLOCK, ADDRESS), GPIO.OUT)
GPIO.setup(DATAOUT, GPIO.IN, GPIO.PUD_UP)

GPIO.setup((RA, RB, CONTROLR), GPIO.OUT)
GPIO.setup((LA, LB, CONTROLL), GPIO.OUT)
GPIO.setup(buzzer,GPIO.OUT)

def analog_read():
    value = [0]*6
    #read channel0 - channel5 AD value
    for j in range(0, 6):
        GPIO.output(CS, GPIO.LOW)
        for i in range(0, 10):
            if i < 4:
                bit = (((j) >> (3-i))&0x01)
                GPIO.output(ADDRESS, bit)

            value[j] <<= 1
            value[j] |= GPIO.input(DATAOUT)
            GPIO.output(CLOCK, GPIO.HIGH)
            GPIO.output(CLOCK, GPIO.LOW)

        GPIO.output(CS, GPIO.HIGH)
        time.sleep(0.0001)
    return value
    #invalid address for channel 0

GPIO.output((RA,RB) , (1,0))
speedR = GPIO.PWM(CONTROLR, 1000)
GPIO.output((LA,LB) , (1,0))
speedL = GPIO.PWM(CONTROLL, 1000)

for pin in pingroup:
    GPIO.add_event_detect(pin,GPIO.FALLING,callback = pin_catched)

while True:
    value = analog_read()
    #print(ranging())
    if value[2]<150 or value[1]<150:
        speedR.start(speed_rate)
        time.sleep(0.1)
        speedR.stop()
    elif value[4]<150 or value[5]<150:
        speedL.start(speed_rate)
        time.sleep(0.1)
        speedL.stop()
    else:
        speedR.start(speed_rate)
        speedL.start(speed_rate)
        time.sleep(0.1)
        speedR.stop()
        speedL.stop()

    distance = ranging()
    #print(distance)
    if distance <=5 or distance >=3000:
        GPIO.output(buzzer,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(buzzer,GPIO.LOW)
        

