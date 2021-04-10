import serial  # to establish com port
import csv     # to r/w .csv
import sys     # to parse argv
from serial.tools import list_ports

ports = list_ports.comports()

dev_port = 0
for port in ports:
    #if port.pid == 11111:
    print(port.name)
    dev_port = "COM12" #port.name

if dev_port == 0:
    print("Device not found, exiting...")
    sys.exit(1)


# now that we have the port, lets open it
ser = serial.Serial(dev_port, baudrate=4000000)
ser.write([170])  # magic number that triggers a dump in firmware  0xA5 is 165
print("connected to Serial Port")

data = ser.read(1000000)  # read 8 megabytes of data, we should not do this

print("read data")
# I think more likely we will be getting things in 1kB chunks
# but need to figure out how we will do it in firmware to fix this part

# now convert bytes to ints and write row-wise to an output csv
data_int = []
for byte in data:
    #data_int.append(int.from_bytes(byte, byteorder='big', signed=True))
    data_int.append(byte)

print("converted data to array")

with open('dump.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(data_int)

print("Succesfully wrote to CSV")
