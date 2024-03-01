import evdev
import serial
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('indev')
parser.add_argument('serdev')
parser.add_argument('outfilename')

args = parser.parse_args()

idev = evdev.InputDevice(args.indev)
ser = serial.Serial(args.serdev, baudrate=115200, timeout=0)
ser.write(b'go')


with open(args.outfilename, 'wb') as f:
    for event in dev.read_loop():
        f.write(bytes(str(event)))
        for line in ser.readline():
            if len(line) == 0:
                break
            if line.endswith(b'\r\n'):
                line = line[:-2]
                f.write(bytes(line))
            else:
                print('invalid serial line', line)
    