import serial
import time

print "Connecting to serial"
ser = serial.Serial("/dev/tty.usbmodemfd121", 115200);

ser.write("\xF0")
time.sleep(0.1)
ser.write("\xF0")
time.sleep(0.1)

ser.write("\xA5\x07\x00\x00\x00\x06\x00\xF2\x01\x00\x10\x00\x05\x20")
time.sleep(1)

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1))

ser.write("\xA5\x07\x00\x00\x00\x0A\x00\xEE\x04\x00\xE8\x01\x08\x00\x02\x00\x00\x00")
time.sleep(1)
ctr = 0
mac = ""

while ser.inWaiting() > 0:
  readByte = ser.read(1)
  print "%#04x" % ord(readByte)
  ctr += 1
  if ctr > 12:
    mac += "%#04x:" % ord(readByte)

ser.write("\xA5\x07\x00\x00\x00\x0A\x00\xEE\x04\x04\xE8\x01\x08\x00\x02\x00\x00\x00")
time.sleep(1)
while ser.inWaiting() > 0:
  print "%#04x " % ord(ser.read(1))

print "MAC address is: " + mac[:-1]
print "Done"
