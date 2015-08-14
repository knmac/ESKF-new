__author__ = 'cig'

# A basic wired streaming mode example
#
# The default mode of communication for a 3-Space Sensor is a call and response paradigm wherein you send a
# command and then receive a response. The sensor also features a streaming mode where it can be instructed to
# periodically send back the response from a command automatically, without any further communication from the host.
#
# This example code is compatible with all 3-Space Sensors connected via USB, serial, or Bluetooth
# Requires 2.0+ firmware features
import serial # 3rd party module download at https://pypi.python.org/pypi/pyserial
import struct
import time
from subprocess import call

COMPORT = "/dev/ttyACM0"
INTERVAL = 20000 # microseconds
DURATION = 0xffffffff # infinite streaming
DELAY = 0 # microseconds
OUTPUT_FILENAME = "sensor_data.txt"

# Start Bytes, indicates the start of a command packet
TSS_START_BYTE = 0xf7
TSS_START_BYTE_WITH_HEADER = 0xf9
# For a full list of streamable commands refer to "Wired Streaming Mode" section in the
# 3-Space Manual of your sensor
TSS_GET_TARED_ORIENTATION_AS_QUATERNION = 0x00
TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = 0x29
TSS_GET_BUTTON_STATE = 0xfa
TSS_NULL = 0xff # No command use to fill the empty slots in "set stream slots"

# For a full list of commands refer to the 3-Space Manual of your sensor
TSS_SET_STREAMING_SLOTS = 0x50
TSS_GET_STREAMING_SLOTS = 0x51
TSS_SET_STREAMING_TIMING = 0x52
TSS_START_STREAMING = 0x55
TSS_STOP_STREAMING = 0x56
TSS_GET_RAW_ACCELERATION = 0x42
TSS_GET_RAW_GYRO = 0x41
TSS_SET_HEADER = 0xdd
TSS_SET_TIMESTAMP = 0x02
TSS_GET_CORRECTED_ACCELERATION = 0x27
TSS_GET_CORRECTED_GYRO = 0x26


# Streaming mode require the streaming slots and streaming timing being setup prior to start streaming
# /////////////////////////////////////////////////////////////////////////////
def setupStreaming(serial_port):
    print("TSS_SET_STREAMING_SLOTS")
    # There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
    # Unused slots should be filled with 0xff so that they will output nothing.
    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_SET_STREAMING_SLOTS)
    write_bytes.append(TSS_GET_RAW_ACCELERATION) # stream slot0
    write_bytes.append(TSS_GET_RAW_GYRO) # stream slot1
    write_bytes.append(TSS_GET_BUTTON_STATE) # stream slot2
    write_bytes.append(TSS_NULL) # stream slot3
    write_bytes.append(TSS_NULL) # stream slot4
    write_bytes.append(TSS_NULL) # stream slot5
    write_bytes.append(TSS_NULL) # stream slot6
    write_bytes.append(TSS_NULL) # stream slot7

    # calculating the checksum (note the checksum doesn't include the start byte)
    write_bytes.append((sum(write_bytes) - write_bytes[0]) % 256)

    # Write the bytes to the serial
    serial_port.write(write_bytes)

    print("TSS_SET_STREAMING_TIMING")

    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_SET_STREAMING_TIMING)

    # The data must be flipped to big endian before sending to sensor
    write_bytes.extend(bytearray(struct.pack('>I', INTERVAL)))
    write_bytes.extend(bytearray(struct.pack('>I', DURATION)))
    write_bytes.extend(bytearray(struct.pack('>I', DELAY)))

    # calculating the checksum (note the checksum doesn't include the start byte
    write_bytes.append((sum(write_bytes) - write_bytes[0]) % 256)

    # Write the bytes to the serial
    serial_port.write(write_bytes)

def setupHeader(serial_port):
    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_SET_HEADER)
    write_bytes.append(TSS_SET_TIMESTAMP)
    write_bytes.append((sum(write_bytes) - write_bytes[0]) % 256)
    # Write the bytes to the serial
    serial_port.write(write_bytes)

# /////////////////////////////////////////////////////////////////////////////
def checkStream(serial_port):
    write_bytes = bytearray()
    write_bytes.append(TSS_START_BYTE)
    write_bytes.append(TSS_GET_STREAMING_SLOTS)
    write_bytes.append(TSS_GET_STREAMING_SLOTS)
    serial_port.write(write_bytes)
    data_struct = struct.Struct('>bbbbbbbb')
    if (serial_port.isOpen()):
        try:
            # Read the bytes returned from the serial
            data_str = serial_port.read(data_struct.size)
            serial_port.flushOutput()

            if len(data_str) != data_struct.size:
                print("Sensor is not answering")

            # The data must be flipped to little endian to be read correctly
            data = data_struct.unpack(data_str)
            #print("================= SAMPLE "+ str(sample_count) +" =========================")
            #dataString = '{0:8.5f} {1:8.5f} {2:8.5f}\n'.format(data[0], data[1], data[2])
            print("Slots: {0: d}, {1: d}, {2: d}, {3: d}, {4: d}, {5: d}, {6: d}, {7: d}".format(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]))         

        except Exception as ex:
            print(ex)
    else:
        print("Serial Not Open")
        
# ////////////////////////////////////main/////////////////////////////////////

# This creates the comport handle and does initial setup like baudrates and timeouts
serial_port = serial.Serial(COMPORT, timeout=1.0, writeTimeout=1.0, baudrate=115200)

setupStreaming(serial_port)
# checkStream(serial_port)
setupHeader(serial_port)

print("TSS_START_STREAMING")
# With parameterless wired commands the command byte will be the same as the checksum
write_bytes = bytearray((TSS_START_BYTE_WITH_HEADER,
                        TSS_START_STREAMING,
                        TSS_START_STREAMING))

# Write the bytes to the serial
serial_port.write(write_bytes)

print("Sensor has started Streaming")

start_time = time.time()
current_time = time.time()
sample_count = 0
# Use Struct in places where the same packing/unpack is used multiple times over
data_struct = struct.Struct('>fffffffB') # format must conform to the expected stream data

try:
    # Read the bytes returned from the serial
    data_str = serial_port.read(data_struct.size)
    serial_port.flushOutput()
except Exception as ex:
    print(ex)

saveEnable = False
fileToWrite = open(OUTPUT_FILENAME, 'w')

while True:
    if (serial_port.isOpen()):
        try:
            current_time = time.time()

            # Read the bytes returned from the serial
            data_str = serial_port.read(data_struct.size)
            serial_port.flushOutput()

            if len(data_str) != data_struct.size:
                print("Sensor is no longer streaming")
                continue

            # Read button state
            data = data_struct.unpack(data_str)
            buttonState = data[7]

            # Process button state
            if buttonState == 1 and saveEnable == False:
                saveEnable = True
                print 'Start streaming'
            elif buttonState == 2:
                write_bytes = bytearray()
                write_bytes.append(TSS_START_BYTE)
                write_bytes.append(TSS_STOP_STREAMING)
                write_bytes.append(TSS_STOP_STREAMING)
                print 'Stop streaming'
                break

            if saveEnable == True:
                sample_count += 1

                print("Time:       {0: 8.3f}".format(data[0]))
                print("Raw Acc:    {0: 8.5f}, {1: 8.5f}, {2: 8.5f}".format(data[1], data[2], data[3]))
                print("Raw Gyro:   {0: 8.5f}, {1: 8.5f}, {2: 8.5f}".format(data[4], data[5], data[6]))
                print("")

                dataString = '{0:8.3f} {1:8.5f} {2:8.5f} {3:8.5f} {4:8.5f} {5:8.5f} {6:8.5f}\n'.format(data[0], data[1], data[2], data[3], data[4], data[5], data[6])
                fileToWrite.write(dataString)
        except Exception as ex:
            print(ex)
    else:
        print("Serial Not Open")
        break


fileToWrite.close()
print("Sample Count={0}".format(sample_count))

# Close the serial
serial_port.close()
