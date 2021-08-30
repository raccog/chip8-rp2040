import serial
import os


ser = serial.Serial('/dev/serial0', baudrate=115200)
ser.write((1).to_bytes(2, 'little'))
filename = 'IBM Logo.ch8'
ser.write(os.path.getsize(filename).to_bytes(2, 'little'))
ser.write(len(filename).to_bytes(1, 'little'))
ser.write(bytes(filename, 'ascii'))
with open('IBM Logo.ch8', 'rb') as f:
    ser.write(f.read())
ser.close()
