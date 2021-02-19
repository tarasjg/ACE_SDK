import serial  # to establish com port
import csv     # to r/w .csv
import sys     # to parse argv
from serial.tools import list_ports

ports = list_ports.comports()
dev_port = 0
for port in ports:
    if port.pid == 11111:
        dev_port = port

if dev_port == 0:
    print("Device not found, exiting...")
    sys.exit(1)


# now that we have the port, lets open it
ser = serial.Serial(dev_port)
ser.write(0xAA)  # magic number that triggers a dump in firmware

data = ser.read(8e+6)  # read 8 megabytes of data, we should not do this

# I think more likely we will be getting things in 1kB chunks
# but need to figure out how we will do it in firmware to fix this part

# now convert bytes to ints and write row-wise to an output csv
data_int = []
for byte in data:
    data_int.append(int.from_bytes(byte, byteorder='big', signed=True))


with open('dump.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerows(data_int)