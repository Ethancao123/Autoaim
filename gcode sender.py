import serial
import time
from datetime import datetime

def command(ser, command):
  start_time = datetime.now()
  ser.write(str.encode(command)) 
#   time.sleep(1)
  print(command)
  while True:
    break
    line = ser.readline()
    print(line)

    if line == 'b\'ok 0\\r\\n\'':
      break

ser = serial.Serial('/dev/ttyUSB0', 115200)
command(ser, "G21\r\n")
time.sleep(1)
command(ser, "G90\r\n")
time.sleep(1)
command(ser, "M82\r\n")
time.sleep(1)
command(ser, "M92 Y17 Z37\r\n")
time.sleep(1)
command(ser, "M203 Y600 Z600\r\n")
time.sleep(1)
command(ser, "G28 Z0\r\n")
while 1:
  command(ser, input() + '\r\n')

# command(ser, "G92 X0 Y0 Z0\r\n")
# command(ser, "G28 X0 Y0\r\n")
# command(ser, "G28 X0\r\n")
# command(ser, "G28 Y0\r\n")
# command(ser, "G28 Z0\r\n")

# Extruder Temp
# command(ser, "M104 S190 T0\r\n") #  start heating T0 to 190 degrees Celsius
# command(ser, "G28\r\n") # Home
# command(ser, "M109 S190 T0\r\n") # wait for T0 to reach 190 degrees before continuing with any other commands

# Bed Temp
# command(ser, "M140 S55\r\n") # heat bed to 50 degrees celsius but do not wait
# command(ser, "G28\r\n") # Home
# command(ser, "M190 S55\r\n") # wait for bed to heat to 50 degrees celsius and wait

# Fan
# command(ser, "M106 S255\r\n") # fan speed full
# command(ser, "M106 S127\r\n") # fan speed about half
# command(ser, "M106 S0\r\n") # turn off fan

# Set Units(does not seem to work on ender 5)
# command(ser, "G20\r\n") # inches
# command(ser, "G21\r\n") # millimeters

# Absolute Mode
# command(ser, "G90\r\n")

# Move
# command(ser, "G0 X7 Y18\r\n") # rapid motion but does not extrude material
# command(ser, "G0 X10 Y10 Z10\r\n") # rapid motion but does not extrude material ender 5 plus is 350 x 350

time.sleep(2)
ser.close()