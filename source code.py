# Configure the serial port for communication with the SIM800L module
list = []

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import socket
import argparse
import geopy.distance
import serial
#import face_recognition
#import picamera
import numpy as np

relay = 17
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay, GPIO.OUT)
GPIO.setwarnings(False)

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

def turn_on():
    GPIO.output(relay, GPIO.HIGH)  # Turn on the relay
    print("Relay turned ON")

def turn_off():
    GPIO.output(relay, GPIO.LOW)  # Turn off the relay
    print("Relay turned OFF")

turn_off()
time.sleep(1)
turn_on()
time.sleep(1)
turn_off()

# Connect to drone
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

# Arm and takeoff to meters
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(3)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_dstance(cord1, cord2):
    return (geopy.distance.geodesic(cord1, cord2).km) * 1000

def goto_location(to_lat, to_long):
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    to_pont = LocationGlobalRelative(to_lat, to_long, curr_alt)
    vehicle.simple_goto(to_pont, groundspeed=8)
    to_cord = (to_lat, to_long)

    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance remaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

def send_command(command):
    ser.write(command.encode() + b'\r\n')
    time.sleep(1)
    response = ser.read_all().decode()
    return response

def setup_sim800l():
    send_command('AT')
    send_command('AT+CMGF=1')
    send_command('AT+CNMI=1,2,0,0,0')

def read_serial():
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        list.append(line)

def main():
    setup_sim800l()
    z = 0
    while z == 0:
        read_serial()
        print(list)
        for i in list:
            if "google" in i:
                z = 1
                break

main()
print("breaked")

lg = list[-1]
lat1s = lg.index("query=") + 6
lat1e = lg.index("%")
longi1s = lg.index("C") + 1

vehicle = connectMyCopter()
time.sleep(2)
ht = 20
arm_and_takeoff(ht)
turn_on()

latitude = float(lg[lat1s:lat1e])
print(latitude)
longitude = float(lg[longi1s:])
print(longitude)

time.sleep(2)
goto_location(latitude, longitude)

time.sleep(10)
turn_off()

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
