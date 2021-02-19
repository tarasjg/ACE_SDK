import serial  # to establish com port
import csv     # to r/w .csv
import sys     # to parse argv


# on linux, we should run lsusb and find the port with the correct PID/VID
# and then run this applet in terminal >python presenter.py -p /dev/ttyx

# format ['presenter.py', '-p', port]
try:
    if sys.argv[1] == '-p':
        port = sys.argv[2]
    else:
        print('Invalid arguments: -p [port]')
        sys.exit()
except IndexError:
    print('Invalid arguments: -p [port]')
    sys.exit()

# now that we have the port, lets open it
ser = serial.Serial(port)
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