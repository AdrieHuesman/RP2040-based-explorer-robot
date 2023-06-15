"""
Obstacle avoiding robot: Pico Bot
Adrie Huesman
Start: 1 Nov 2022
Last update: 10 Dec 2022
Hardware
2 120:1 Mini Plastic Gearmotors HP: https://www.pololu.com/product/1520
Pololu Mini Plastic Gearmotor Bracket Pair - Wide: https://www.pololu.com/product/2680
Pololu Wheel 70×8mm Pair - Black: https://www.pololu.com/product/1425 
Magnetic Encoder Pair Kit, 12 CPR, 2.7-18V: https://www.pololu.com/product/1523
Swivel wheel: https://www.gamma.nl/assortiment/zwenkwiel-tpe-met-plaatbevestiging-25mm-tot-15kg/p/B328710
Battery holder 4 AA
4 NiMH batteries AA
Maker Pi RP2040 board: https://www.cytron.io/p-maker-pi-rp2040-simplifying-robotics-with-raspberry-pi-rp2040
270 degrees servo: https://www.bitsandparts.nl/Servo-motor-analoog-Micro-Servo-9g-SG90-270%C2%B0-p1919329
Ultrasonic sensor RCLW-1601: https://www.tinytronics.nl/shop/nl/sensoren/afstand/ultrasonische-sensor-rcwl-1601
2 Pololu Distance Sensors with Pulse Width Output: https://www.pololu.com/product/4064 
Robot chassis 10 by 15 cm, two levels, made from 4 mm plywood: https://quartel.nl/product/berken-triplex-25-x-100-4-0mm-1mtr-bet34-0/
Rev A Basic functionality.
Rev B Slow servo movement.
Rev C MFO = f(USdistance).
Rev D Write data to file.
Rev E Use of machine.time_pulse_us for Ping, leftIR and rightIR.
Rev F Replaced US sensor/servo with VL53L1X sensor placed at the back of the robot to increase FOV
Rev G Added US sensor at the back of the robot to double FOV, so there are two sensors at the back
Rev H US sensor replaced by URM09 US sensor
Rev I VL53L1X and URM09 US sensor placed at the front
Rev J MFO was given a gradual stop at exit
"""

# Import libraries
import machine
import utime
import random
from neopixel import NeoPixel
from vl53l1x import VL53L1X # taken from https://github.com/drakxtwo/vl53l1x_pico
utime.sleep(2) # for proper booting!

# Setup GPIO pins
# DC motors
M1A = machine.PWM(machine.Pin(8))
M1B = machine.PWM(machine.Pin(9))
M2A = machine.PWM(machine.Pin(10))
M2B = machine.PWM(machine.Pin(11))
M1A.freq(1000)
M1B.freq(1000)
M2A.freq(1000)
M2B.freq(1000)
# US sensor
trigger = machine.Pin(4, machine.Pin.OUT) # send trigger
echo = machine.Pin(5, machine.Pin.IN) # get echo back
# Button
button1 = machine.Pin(20, machine.Pin.IN) # switch has external pull-up to 3.3v via 10 kohm
# Neopixel
pin = machine.Pin(18, machine.Pin.OUT)   # set GPIO18 to drive NeoPixels
np = NeoPixel(pin, 2) # create NeoPixel driver for 2 pixels
# Speaker
speaker = machine.PWM(machine.Pin(22))
# IR sensors
leftIRpin = machine.Pin(6, machine.Pin.IN)
rightIRpin = machine.Pin(26, machine.Pin.IN)
# VL53L1X sensor
sda=machine.Pin(16)
scl=machine.Pin(17)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
distance = VL53L1X(i2c)

# Functions and interrupts
def leftmotor(speed):   # speed must be between -65535 and 65535
    if speed >= 0:   
        M1A.duty_u16(0)     
        M1B.duty_u16(speed)
    else:
        M1A.duty_u16(-speed)
        M1B.duty_u16(0)
        
def rightmotor(speed):   # speed must be between -65535 and 65535
    if speed >= 0:   
        M2A.duty_u16(0)     
        M2B.duty_u16(speed)
    else:
        M2A.duty_u16(-speed)
        M2B.duty_u16(0)
        
# Global values
leftsteps = 0
rightsteps = 0

# Interrupt Service Routine for left motor
def leftencoder(change):
    global leftsteps
    leftsteps += 1

# Define leftencoderA as being connected to GP7
leftencoderA = machine.Pin(7, machine.Pin.IN) # encoder has external pull-up to 3.3v via 10 kohm

# Associate the falling value on the input pin with the callback function
leftencoderA.irq(handler = leftencoder, trigger = machine.Pin.IRQ_FALLING)

# Interrupt Service Routine for right motor
def rightencoder(change):
    global rightsteps
    rightsteps += 1

# Define rightencoderA as being connected to GP0
rightencoderA = machine.Pin(0, machine.Pin.IN) # encoder has external pull-up to 3.3v via 10 kohm

# Associate the falling value on the input pin with the callback function
rightencoderA.irq(handler = rightencoder, trigger = machine.Pin.IRQ_FALLING)

def move(direc, steps, aspeed):
    global leftsteps
    leftsteps = 0
    global rightsteps
    rightsteps= 0

    if direc == 'F':
        lfactor = 1
        rfactor = 1

    if direc == 'B':
        lfactor = -1
        rfactor = -1
        
    if direc == 'L':
        lfactor = -1
        rfactor = 1

    if direc == 'R':
        lfactor = 1
        rfactor = -1
        
    leftmotor(lfactor*aspeed)
    rightmotor(rfactor*aspeed)
  
    while leftsteps < steps: # and (rightsteps < steps):
        error = leftsteps - rightsteps
        paction = int(200.0*error)
        leftmotor(lfactor*(aspeed - paction))
        rightmotor(rfactor*(aspeed + paction))
        utime.sleep(0.005)
        
    leftmotor(0)
    rightmotor(0)
    
    #print('steps:', steps)
    #print('left:', leftsteps)
    #print('right:', rightsteps)
    #print('     ')

def playtone(frequency):
    speaker.duty_u16(32768) # 50% dutycycle
    speaker.freq(frequency)
    utime.sleep(0.1)

def bequiet():
    speaker.duty_u16(0)
    
def leftIR():
    timepassed = machine.time_pulse_us(leftIRpin, 1, 35000) # time-out does not limit distance!
    if timepassed < 999 or timepassed > 1999:
        distance = 301
    else:
        distance = round(0.4*(timepassed - 1000), 1)
    return distance # in cm

def rightIR():
    timepassed = machine.time_pulse_us(rightIRpin, 1, 35000) # time-out does not limit distance!
    if timepassed < 999 or timepassed > 1999:
        distance = 301
    else:
        distance = round(0.4*(timepassed - 1000), 1)
    return distance # in cm

def ping():
    trigger = machine.Pin(5, machine.Pin.OUT)
    trigger.value(1)
    utime.sleep_us(10)
    trigger.value(0)
    trigger = machine.Pin(5, machine.Pin.IN)
    timepassed = machine.time_pulse_us(trigger, 1, 35000)
    if timepassed < 0:
        distance = 999
    else:
        distance = round((timepassed * 0.0343)/2, 1)
    return distance

# Tuning
# Limits
Climit = 30.0
RLlimit = 20.0
# MFO state
MFOspeed = 20000
# Neopixels
bright = 50 # max 255

# Main code
state = 'INI'
while True:
    if state == 'INI':
        np[0] = (bright, bright, bright)  # set right pixel to white
        np[1] = (bright, bright, bright)  # set left  pixel to white
        np.write()                        # write data to all pixels
        leftmotor(0)
        rightmotor(0)
        while button1.value():
            utime.sleep(0.1)
        utime.sleep(1.0)
        file = open("temps.txt", "w")
        state = 'SAM'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels
        
    elif state == 'SAM':
        np[0] = (0, 0, bright)  # set right pixel to blue
        np[1] = (0, 0, bright)  # set left  pixel to blue
        np.write()              # write data to all pixels
        leftmotor(0)
        rightmotor(0)
        utime.sleep(1.0)
        IRdistance = (distance.read())/10
        USdistance = ping()
        centdistance = min(IRdistance, USdistance)
        IRleftdist = leftIR()
        IRrightdist = rightIR()
        if centdistance < Climit:
            Cprox = 1
        else:
            Cprox = 0
        if IRleftdist < RLlimit:
            Lprox = 1
        else:
            Lprox = 0
        if IRrightdist < RLlimit:
            Rprox = 1
        else:
            Rprox = 0
        if Cprox == 0 and Lprox == 0 and Rprox == 0:
            state = 'MFO'
        if Cprox == 1 and Lprox == 0 and Rprox == 0:
            state = 'T90'
        if Lprox == 1 and Rprox == 0:
            state = 'T15'
        if Lprox == 0 and Rprox == 1:
            state = 'T15'
        if Lprox == 1 and Rprox == 1:
            state = 'ESC'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels
            
    elif state == 'T90':
        np[0] = (bright, 0, bright)  # set right pixel to magenta
        np[1] = (bright, 0, bright)  # set left  pixel to magenta
        np.write()                   # write data to all pixels
        for i in range(1, 6):
            freq = random.randint(300, 4000)
            playtone(freq)
        bequiet()
        USldistance = leftIR()
        USrdistance = rightIR()
        if USldistance > USrdistance:
            move('L', 174, 20000) # 174 steps should be 90 degrees
        else:
            move('R', 174, 20000) # 174 steps should be 90 degrees
        state = 'SAM'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels
        
    elif state == 'MFO':
        np[0] = (0, bright, 0)  # set right pixel to green
        np[1] = (0, bright, 0)  # set left  pixel to green
        np.write()              # write data to all pixels
        leftsteps = 0
        rightsteps = 0
        lastmeas = 0
        leftmotor(MFOspeed)
        rightmotor(MFOspeed)
        while state == 'MFO':
            error = leftsteps - rightsteps
            paction = int(200.0*error)
            leftmotor(MFOspeed - paction)
            rightmotor(MFOspeed + paction)
            if (leftsteps - lastmeas) > 85: # take meas. each 5 cm
                lastmeas = leftsteps
                IRdistance = (distance.read())/10
                USdistance = ping()
                centdistance = min(IRdistance, USdistance)
                IRleftdist = leftIR()
                IRrightdist = rightIR()
                file.write(str(centdistance) + "\n")
                file.flush()
                if centdistance > 80.0:
                    MFOspeed = int(1.05*MFOspeed)
                    if MFOspeed > 32000:
                        MFOspeed = 32000
                if centdistance < 80.0:
                    MFOspeed = int(0.95*MFOspeed)
                    if MFOspeed < 20000:
                        MFOspeed = 20000
                if centdistance < Climit:
                    Cprox = 1
                else:
                    Cprox = 0
                if IRleftdist < RLlimit:
                    Lprox = 1
                else:
                    Lprox = 0
                if IRrightdist < RLlimit:
                    Rprox = 1
                else:
                    Rprox = 0
                if Cprox == 0 and Lprox == 0 and Rprox == 0:
                    state = 'MFO'
                else:
                    for i in range(1, 6):  # Do a gradual stop
                        MFOspeed = int(0.80*MFOspeed)
                        if MFOspeed < 10000:
                            MFOspeed = 10000
                        leftmotor(MFOspeed)
                        rightmotor(MFOspeed)
                        utime.sleep(0.1)
                    leftmotor(0)
                    rightmotor(0)
                    MFOspeed = 20000
                    state = 'SAM'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels
        
    elif state == 'T15':
        np[0] = (bright, bright, 0)  # set right pixel to yellow
        np[1] = (bright, bright, 0)  # set left  pixel to yellow
        np.write()                   # write data to all pixels
        if Lprox == 1:
            move('R', 29, 20000) # 174 steps should be 90 degrees
        else:
            move('L', 29, 20000) # 174 steps should be 90 degrees
        state = 'SAM'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels
        
    elif state == 'ESC':
        np[0] = (bright, 0, 0)  # set right pixel to red
        np[1] = (bright, 0, 0)  # set left  pixel to re
        np.write()              # write data to all pixels
        move('B', 174, 20000)
        move('R', 348, 20000) # 174 steps should be 90 degrees
        state = 'SAM'
        np[0] = (0, 0, 0)  # set right pixel to off
        np[1] = (0, 0, 0)  # set left  pixel to off
        np.write()         # write data to all pixels        