import serial
import time

print "Connecting to serial"
ser = serial.Serial("/dev/tty.usbmodemfa131", 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, 5)
time.sleep(3)
ser.flushInput()


# Initial handshake
ser.write("\xF0")
time.sleep(0.1)
ser.write("\xF0")
time.sleep(0.1)

ser.write("\xA5\x07\x00\x00\x00\x06\x00\xF2\x01\x00\x10\x00\x05\x20")
time.sleep(1)
# Returns  \xA5\x07\x00\x00\x00\x06\x00\xF2\x11\x04\x05\x00\x03\x01

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1)),
print ""

ser.write("\xA5\x07\x00\x00\x00\x06\x00\xF2\x01\x80\x00\x02\x08\x20")
time.sleep(1)
# Returns  \xA5\x07\x00\x00\x00\x06\x00\xF2\x11\x04\x00\x00\x00\x00

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1)),
print ""

ser.write("\xA5\x07\x00\x00\x00\x06\x00\xF2\x01\x84\x00\x02\x08\x20")
time.sleep(1)
# Returns  \xA5\x07\x00\x00\x00\x06\x00\xF2\x11\x04\x00\x00\x00\x00

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1)),
print ""

ser.write("\xA5\x07\x00\x00\x00\x06\x00\xF2\x01\x88\x00\x02\x08\x20")
time.sleep(1)
# Returns  \xA5\x07\x00\x00\x00\x06\x00\xF2\x11\x04\x44\x4C\x4F\x4B

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1)),
print ""

# Begin flash of app2
sequenceNumber = 0;

ser.write("\xA5\x02\x00\x00\x00")
if partialFragment:
  ser.write("\xB5\x04\x44")
else:
  ser.write("\xE4\x05\x14")

ser.write("\x01")
if sequenceNumber < 16:
  ser.write("0")
ser.write(hex(sequenceNumber)[2:])

ser.write("\x79\xCC\x01\x00\xD8\x05\x00\x00\x00\x04")
time.sleep(1)
# Returns  \xA5\x07\x00\x00\x00\x06\x00\xF2\x11\x04\x44\x4C\x4F\x4B

while ser.inWaiting() > 0:
  print "%#04x" % ord(ser.read(1)),
print ""

print "Done"
