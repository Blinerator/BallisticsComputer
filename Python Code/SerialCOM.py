import serial
import io
import time
import math
import re
import parse
def write_to_arduino(yaw,pitch,joystick_enable,automated_motor_enable,COMPRT_a):
    
#the arduino is constantly checking for a series of values to be sent.  The format is pre-defined in arduino code

    angles = f"{yaw},{pitch},{joystick_enable},{automated_motor_enable},"

    arduino = serial.Serial(COMPRT_a,115200, bytesize=8,stopbits=1,timeout=1)
    arduino.write(bytes(angles,'utf-8'))
    time.sleep(0.1)

    reset=f"{yaw},{pitch},{joystick_enable},0,"
    arduino.write(bytes(reset,'utf-8'))

    return

def read_arduino_sensors(COMPRT_a):
    #control panel has a switch that makes arduino send roll and pitch values over serial
    
    arduino = serial.Serial(COMPRT_a,115200, bytesize=8,stopbits=1,timeout=1)

    raw=arduino.readline()
    decode = re.findall(r'\b\d+\b',str(raw.decode('utf')))
    normalized_pitch=int(decode[0]) #accelerometer directly returns this value
    roll=int(decode[1]) #accelerometer directly returns this value
    print(roll)
    print(normalized_pitch)
    #print(velocity)
    return normalized_pitch, roll

def takerange(COMPRT_b):
    
#lazer rangefinder returns range when byte sequence is sent to it.  Sequence is specific to this model.
#Note that the LRF never stops taking range unless stop_takerange() is called.

    ser = serial.Serial(COMPRT_b,115200, bytesize=8,stopbits=1,timeout=1)

    packet = bytearray()
    packet.append(0x0D)
    packet.append(0x0A)
    packet.append(0x4F)
    packet.append(0x4E)
    packet.append(0x0D)
    packet.append(0x0A)
    ser.write(packet)

    s = str(ser.readline())
    print(s)
    s = parse.p(s)
    return s

def stop_takerange(COMPRT_b):
    ser = serial.Serial(COMPRT_b,115200, bytesize=8,stopbits=1,timeout=1)
    
    packet2 = bytearray()
    packet2.append(0x0D)
    packet2.append(0x0A)
    packet2.append(0x4F)
    packet2.append(0x46)
    packet2.append(0x46)
    packet2.append(0x0D)
    packet2.append(0x0A)
    ser.write(packet2)
