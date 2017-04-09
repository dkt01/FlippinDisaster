import sys
import os
import time
import Queue
import threading
import math
import copy

import atexit
import traceback
import evdev
import serial

import RPi.GPIO as gpio
from xbee import XBee

ROBOT_ADDR = '\x00\x01'
TX_OPT = '\x01'

TIMEOUT_CONTROLLER = 0.2
TIMEOUT_COMMS      = 0.2

POWER_LED = 3
CONTROLLER_LED = 5
COMMS_LED = 7

LED_OFF   = 0
LED_ON    = 1
LED_BLINK = 2

m_powerQueue      = Queue.Queue()
m_controllerQueue = Queue.Queue()
m_commsQueue      = Queue.Queue()

AXIS_ZERO = 128
PRESSURE_ZERO = 0
ACCEL_ZERO = 0
GYRO_ZERO = 0

KEYCODE_MAP =   {
                    0   : "AXIS_LX",
                    1   : "AXIS_LY",
                    2   : "AXIS_RX",
                    5   : "AXIS_RY",
                    44  : "PRESSURE_UP",
                    45  : "PRESSURE_RIGHT",
                    46  : "PRESSURE_DOWN",
                    47  : "PRESSURE_LEFT",
                    48  : "PRESSURE_L2",
                    49  : "PRESSURE_R2",
                    50  : "PRESSURE_L1",
                    51  : "PRESSURE_R1",
                    52  : "PRESSURE_TRIANGLE",
                    53  : "PRESSURE_O",
                    54  : "PRESSURE_X",
                    55  : "PRESSURE_SQUARE",
                    59  : "ACCEL_X",
                    60  : "ACCEL_Y",
                    61  : "ACCEL_Z",
                    62  : "GYRO_Z",
                    288 : "BUTTON_SELECT",
                    289 : "BUTTON_L3",
                    290 : "BUTTON_R3",
                    291 : "BUTTON_START",
                    292 : "BUTTON_UP",
                    293 : "BUTTON_RIGHT",
                    294 : "BUTTON_DOWN",
                    295 : "BUTTON_LEFT",
                    296 : "BUTTON_L2",
                    297 : "BUTTON_R2",
                    298 : "BUTTON_L1",
                    299 : "BUTTON_R1",
                    300 : "BUTTON_TRIANGLE",
                    301 : "BUTTON_O",
                    302 : "BUTTON_X",
                    303 : "BUTTON_SQUARE",
                    704 : "BUTTON_PS",
                }

m_controllerState = {
                        "BUTTON_SELECT"     : 0,
                        "BUTTON_L3"         : 0,
                        "BUTTON_R3"         : 0,
                        "BUTTON_START"      : 0,
                        "BUTTON_UP"         : 0,
                        "BUTTON_RIGHT"      : 0,
                        "BUTTON_DOWN"       : 0,
                        "BUTTON_LEFT"       : 0,
                        "BUTTON_L2"         : 0,
                        "BUTTON_R2"         : 0,
                        "BUTTON_L1"         : 0,
                        "BUTTON_R1"         : 0,
                        "BUTTON_TRIANGLE"   : 0,
                        "BUTTON_O"          : 0,
                        "BUTTON_X"          : 0,
                        "BUTTON_SQUARE"     : 0,
                        "BUTTON_PS"         : 0,

                        "AXIS_LX"           : AXIS_ZERO,
                        "AXIS_LY"           : AXIS_ZERO,
                        "AXIS_RX"           : AXIS_ZERO,
                        "AXIS_RY"           : AXIS_ZERO,

                        "PRESSURE_UP"       : PRESSURE_ZERO,
                        "PRESSURE_RIGHT"    : PRESSURE_ZERO,
                        "PRESSURE_DOWN"     : PRESSURE_ZERO,
                        "PRESSURE_LEFT"     : PRESSURE_ZERO,
                        "PRESSURE_L2"       : PRESSURE_ZERO,
                        "PRESSURE_R2"       : PRESSURE_ZERO,
                        "PRESSURE_L1"       : PRESSURE_ZERO,
                        "PRESSURE_R1"       : PRESSURE_ZERO,
                        "PRESSURE_TRIANGLE" : PRESSURE_ZERO,
                        "PRESSURE_O"        : PRESSURE_ZERO,
                        "PRESSURE_X"        : PRESSURE_ZERO,
                        "PRESSURE_SQUARE"   : PRESSURE_ZERO,

                        "ACCEL_X"           : ACCEL_ZERO,
                        "ACCEL_Y"           : ACCEL_ZERO,
                        "ACCEL_Z"           : ACCEL_ZERO,

                        "GYRO_Z"            : GYRO_ZERO,
                    }

CONTROLLER_ZERO = copy.deepcopy(m_controllerState)

SERIAL_BUTTONS = [
                    "BUTTON_SELECT",
                    "BUTTON_L3",
                    "BUTTON_R3",
                    "BUTTON_START",
                    "BUTTON_UP",
                    "BUTTON_RIGHT",
                    "BUTTON_DOWN",
                    "BUTTON_LEFT",
                    "BUTTON_L2",
                    "BUTTON_R2",
                    "BUTTON_L1",
                    "BUTTON_R1",
                    "BUTTON_TRIANGLE",
                    "BUTTON_O",
                    "BUTTON_X",
                    "BUTTON_SQUARE",
                    "BUTTON_PS",
                 ]
SERIAL_AXES = [
                "AXIS_LX",
                "AXIS_LY",
                "AXIS_RX",
                "AXIS_RY"
              ]

def deluminate(leds):
    for led in leds:
        gpio.output(led,gpio.LOW)

def ledControl(powerQueue, controllerQueue, commsQueue, runEvent):
    loopCount = 0
    powerState = LED_OFF
    controllerState = LED_OFF
    commsState = LED_OFF

    while(runEvent.is_set()):
        try:
            for i in range(10):
                powerState = powerQueue.get(block=False)
        except Queue.Empty:
            pass

        try:
            for i in range(10):
                controllerState = controllerQueue.get(block=False)
        except Queue.Empty:
            pass

        try:
            for i in range(10):
                commsState = commsQueue.get(block=False)
        except:
            pass

        if (powerState == LED_ON) or ( (powerState == LED_BLINK) and (loopCount % 2 == 0) ):
            gpio.output(POWER_LED,gpio.HIGH)
        else:
            gpio.output(POWER_LED,gpio.LOW)

        if (controllerState == LED_ON) or ( (controllerState == LED_BLINK) and (loopCount % 2 == 0) ):
            gpio.output(CONTROLLER_LED,gpio.HIGH)
        else:
            gpio.output(CONTROLLER_LED,gpio.LOW)

        if (commsState == LED_ON) or ( (commsState == LED_BLINK) and (loopCount % 2 == 0) ):
            gpio.output(COMMS_LED,gpio.HIGH)
        else:
            gpio.output(COMMS_LED,gpio.LOW)

        loopCount += 1
        time.sleep(0.25)

def controllerControl(controllerQueue, controllerState, runEvent):
    latestEvent = time.clock()
    latestMode  = None
    ps = None
    # Outer loop handles disconnected controller
    while(runEvent.is_set()):
        devices = evdev.list_devices()

        if len(devices) > 0:
            print "New controller @", devices[0]
            ps = evdev.InputDevice(devices[0])

        # Inner loop handles normal input events
        while(runEvent.is_set() and len(evdev.list_devices()) > 0):
            try:
                for event in ps.read():
                    if event.type != evdev.ecodes.EV_SYN and event.code in KEYCODE_MAP:
                        controllerState[KEYCODE_MAP[event.code]] = event.value

                latestEvent = time.clock()
            except IOError:
                pass

            newMode = LED_ON
            if (time.clock() - latestEvent >= TIMEOUT_CONTROLLER):
                # print "Timeout", time.clock() - latestEvent
                controllerState.update(copy.deepcopy(CONTROLLER_ZERO)) # Zero out for safety
                newMode = LED_BLINK

            if newMode != latestMode:
                controllerQueue.put(newMode)
                latestMode = newMode

            time.sleep(0.02) # Limit loop rate to 50Hz

        newMode = LED_OFF
        controllerState.update(copy.deepcopy(CONTROLLER_ZERO)) # Zero out for safety

        if newMode != latestMode:
            # print "LOST_CONTROLLER"
            controllerQueue.put(newMode)
            latestMode = newMode

        time.sleep(0.1) # Limit discovery loop rate to 10Hz

# Receive Acks from Arduino
def receiveData(xbee, ackTime, runEvent):
    oldCB = xbee._callback
    oldTC = xbee._thread_continue
    xbee._callback=True
    xbee._thread_continue=lambda: runEvent.is_set()
    try:
        while(runEvent.is_set()):
            data = xbee.wait_read_frame()
            ackTime[0] = time.clock()
    except Exception:
        pass # Exception will be raised on event interrupt
    finally:
        # This prevents weirdness during xbee shutdown
        xbee._thread_continue=oldTC
        xbee._callback=oldCB

def serializeState(controllerState):
    buttonBytes = '\x00' * int(math.ceil(float(len(SERIAL_BUTTONS)) / 8.0))
    axisBytes = '\x00' * len(SERIAL_AXES)

    buttonBytes = list(buttonBytes)
    axisBytes = list(axisBytes)

    for btn in range(len(SERIAL_BUTTONS)):
        byteNum = int(math.floor(btn/8.0))
        bitNum = btn % 8
        val = controllerState[SERIAL_BUTTONS[btn]]
        buttonBytes[byteNum] = chr(ord(buttonBytes[byteNum]) | (val << bitNum))

    for axs in range(len(SERIAL_AXES)):
        val = controllerState[SERIAL_AXES[axs]]
        axisBytes[axs] = chr(val)

    buttonBytes = "".join(buttonBytes)
    axisBytes = "".join(axisBytes)

    return buttonBytes + axisBytes

def commControl(commsQueue, controllerState, runEvent):
    m_ser = None
    m_xbee = None
    latestMode = LED_OFF
    latestAck = [0]
    commsQueue.put(latestMode)
    thread_read = None
    runReadEvent = threading.Event()
    runReadEvent.clear()

    while(runEvent.is_set()):
        serDevs = [ f for f in os.listdir('/dev') if 'ttyUSB' in f ]
        if len(serDevs) == 0:
            time.sleep(0.1) # Limit discovery loop rate to 10Hz
            continue
        try:
            print "New XBee @",serDevs[0]
            m_ser = serial.Serial('/dev/'+serDevs[0])
            m_xbee = XBee(m_ser, escaped=True)

            runReadEvent.set()
            thread_read = threading.Thread(target=receiveData, args=(m_xbee,latestAck,runReadEvent))
            thread_read.start()

            while(runEvent.is_set()):
                message = serializeState(controllerState)
                m_xbee.send("tx", dest_addr=ROBOT_ADDR, frame_id='\x00', options=TX_OPT, data=message)

                newMode = LED_ON
                if(time.clock() - latestAck[0] >= TIMEOUT_COMMS):
                    newMode = LED_BLINK

                if(newMode != latestMode):
                    commsQueue.put(newMode)
                    latestMode = newMode

                time.sleep(0.05) # Limit loop rate to 20Hz
            runReadEvent.clear()
            m_xbee.halt()
        except Exception as e:
            # traceback.print_exc()
            newMode = LED_OFF
            if newMode != latestMode:
                commsQueue.put(newMode)
                latestMode = newMode
            if(thread_read != None):
                runReadEvent.clear()
            # if(m_xbee != None and m_xbee.isAlive):
            #     m_xbee.halt()
            time.sleep(0.1) # Limit discovery loop rate to 10Hz

runEvent = threading.Event()
runEvent.set()

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)
gpio.setup(POWER_LED,gpio.OUT)
gpio.setup(CONTROLLER_LED,gpio.OUT)
gpio.setup(COMMS_LED,gpio.OUT)

atexit.register(deluminate, [POWER_LED,CONTROLLER_LED,COMMS_LED])

thread_ledControl = threading.Thread(target=ledControl, args=(m_powerQueue,m_controllerQueue,m_commsQueue,runEvent))
thread_controllerControl = threading.Thread(target=controllerControl, args=(m_controllerQueue,m_controllerState,runEvent))
thread_commControl = threading.Thread(target=commControl, args=(m_commsQueue,m_controllerState,runEvent))

thread_ledControl.start()
thread_controllerControl.start()
thread_commControl.start()

m_powerQueue.put(LED_ON)

try:
    while(True):
        time.sleep(0.1)
except KeyboardInterrupt:
    runEvent.clear()
    thread_commControl.join()
    thread_controllerControl.join()
    thread_ledControl.join()

if threading.activeCount() > 1:
    print "STILL RUNNING THREADS:",threading.enumerate()
