#!/usr/bin/env python
import serial


usbport = 'COM9'



# Set up serial baud rate
ser = serial.Serial(usbport, 9600, timeout=1)

        
def move(servo, angle):

    if (0 <= angle <= 180):
        ser.write(255)
        ser.write(servo)
        ser.write(angle)
    else:
        print("Servo angle must be an integer between 0 and 180.\n")


def init():
    move(1,90)
    move(2,90)
    move(3,90)
    move(4,90)
    move(5,90)
    move(6,90)

init()

while True:
    a,b=input("Enter servo number, angle: ")
    move(a,b)