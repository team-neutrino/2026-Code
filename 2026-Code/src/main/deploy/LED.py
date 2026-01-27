import board
import neopixel
from colorutils import Color

import sys, time
import logging

from networktables import NetworkTables

logging.basicConfig(level=logging.DEBUG)

NetworkTables.setDashboardMode(1735)
NetworkTables.initialize(server=ip)

def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
led = NetworkTables.getTable("/LED")
color = led.getAutoUpdateValue("color", "")
state = led.getAutoUpdateValue("state", "")

color.addListener(valueColorChanged, NetworkTables.NotifyFlags.UPDATE)
state.addListener(valueStateChanged, NetworkTables.NotifyFlags.UPDATE)

ip = "10.39.28.2"

pixels = neopixel.NeoPixel(board.D18, 298, auto_write=False)

# initialize variables for colors
global targetColorRGB
global newColorRGB
targetColorRGB = [0, 0, 0]
newColorRGB = [0, 0, 0]

# initialize variables for blinking 
blink = False
previousBlinkTime = 0
amount = 0

# define colors 
white = (64, 64, 64) # blink countdown
red = (64, 0, 0) # red hub
orange = (84, 18, 0) # full hopper
yellow = (64, 64, 0)
green = (0, 64, 0)
blue = (65, 105, 255) # blue hub
purple = (64, 0, 64) # default to purple (during auton, transition shift, endgame)
black = (0, 0, 0)

def setTargetColor(rgb):
    global targetColorRGB
    targetColorRGB = rgb

def setNewColor(rgb): # set new color and ensure all rgb values btwn 0 and 255
    global newColorRGB
    newColorRGB[0] = min(255,max(rgb[0], 0))
    newColorRGB[1] = min(255,max(rgb[1], 0))
    newColorRGB[2] = min(255,max(rgb[2], 0))

def blinkColor(rgb):
    global blink
    global previousBlinkTime
    global amount # how many times the LEDs blink

    timePassed = time.time() - previousBlinkTime

    # on for half a second and off for half a second
    if timePassed < 0.5:
        setNewColor(rgb)
    elif timePassed > 0.5 and timePassed < 1:
        setNewColor(black)
    else:
        amount -= 1
        if amount == 0:
            blink = False
            setNewColor(rgb) 

def ramp(rgb): #gradually change to new color 
    global newColorRGB

    rgb2 = newColorRGB
    rate = 10 # how much added/subtracted to rgb value 

    if(rgb2[0] < rgb[0]):
       rgb2[0] += rate
    if(rgb2[0] > rgb[0]):
       rgb2[0] -= rate
    if(rgb2[1] < rgb[1]):
        rgb2[1] += rate
    if(rgb2[1] > rgb[1]):
        rgb2[1] -= rate
    if(rgb2[2] < rgb[2]):
        rgb2[2] += rate
    if(rgb2[2] > rgb[2]):
        rgb2[2] -= rate
    setNewColor(rgb2)

def valueColorChanged(table, key, value, isNew):
    print("valueColorChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global targetColorRGB

    if value == "white":
        setTargetColor(white)
    if value == "red":
        setTargetColor(red)
    if value == "orange":
        setTargetColor(orange)
    if value == "yellow":
        setTargetColor(yellow)
    if value == "green":
        setTargetColor(green)
    if value=="blue":
        setTargetColor(blue)
    if value == "purple":
        setTargetColor(purple)
    if value == "black":
        setTargetColor(black)

    for t in range(19,0,-1):
        for i in range(t, t + 19):
            if value == "rainbow":
                c = Color(hsv=((i-t) * (360/19), 1, 1))
                if(i < 19):
                    pixels[i] = (c.red, c.green, c.blue)
                else:
                    pixels[i - 19] = (c.red, c.green, c.blue)
                    

def valueStateChanged(table, key, value, isNew):
    print("valueStateChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    global blink
    global previousBlinkTime
    global amount

    if value == "blink":
        blink = True
        previousBlinkTime = time.time()
        amount = 5
    # if value== "blinktwice":
    #     blink = True
    #     previousBlinkTime = time.time()
    #     amount = 2 
    if value == "solid":
        blink = False

# def printRGB():
    # print("rc %d" % targetColorRGB[0])
    # print("gc %d" % targetColorRGB[1])
    # print("bc %d" % targetColorRGB[2])
    # print(" ")
    # print("rp %d" % newColorRGB[0])
    # print("gp %d" % newColorRGB[1])
    # print("bp %d" % newColorRGB[2])
    # print(" ")

while True:
    time.sleep(0.01)
    ramp(targetColorRGB) # change to target color 
    
    if (blink):
        blinkColor(newColorRGB)
        
    pixels.fill((newColorRGB[0],newColorRGB[1], newColorRGB[2]))
    pixels.show()