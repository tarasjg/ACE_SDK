import csv
import statistics

#Globals
accel_data = list()
numSamples = 0
channels = [[] for i in range(8)]


"""compute the 2's complement of int value val"""
def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def parse_accel_data(fname):
    with open(fname) as in_file:
        csv_reader = csv.reader(in_file, delimiter=',')
        in_accel_sect = False
        prev1 = -1
        prev2 = -1
        msb = True
        data = 0
        xyz = list()

        for row in csv_reader:
            for elem in row:
                byte = int(elem)

                # Each entry is ((MSB << 8) | LSB) >> 4 converted to g's
                if in_accel_sect:
                    if msb:
                        data = byte << 8
                        msb = False
                    else:
                        data = (data | byte) >> 4
                        data = twos_comp(data, 12) * .1  # convert to g's
                        xyz.append(data)
                        msb = True

                    if len(xyz) == 3:
                        accel_data.append(xyz)
                        xyz = list()
                
                # Detects start of accel section
                if byte == 255 and prev1 == 255 and prev2 == 255:
                    in_accel_sect = True
                    xyz = list()

                # Detects start of front end
                if byte == 192 and prev1 == 0 and prev2 == 0:
                    in_accel_sect = False
                
                prev2 = prev1
                prev1 = byte


def parse_eeg(fname):
    #init
    flag = False
    count = 0
    idx = 0
    #Fill out the data for each channel, 8 channels, each channels gets 3 bytes of data for each read (24 total bits)
    with open('dump.csv', 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for data in csv_reader:
            for col in range(len(data)):
            #The c0 00 00 flag was set and we are looking at data
                if (flag and (idx + 23) <= len(data)):
                    for i in range(len(channels)):
                        channels[i].append(int(data[idx]))
                        channels[i].append(int(data[idx+1]))
                        channels[i].append(int(data[idx+2]))
                        idx = idx + 3
                    flag = False
                else:
                    #check for the flag
                    if (data[col] == '192' and (col+2) <= len(data)-1 and data[col+1] == '0' and data[col+2] == '0'):
                        flag = True
                        idx = col + 3

    #number of samples taken times 3 = total number of bytes for a channel collected
    numSamples = int(len(channels[0])/3)
    vref = 2.5
    gain = 24
    LSB = ((2 * vref)/ gain) / (2**24 - 1)
    channelsV = [[] for i in range(8)]

    for ch in range(8):
        for i in range(numSamples):
            idx = i * 3
            x = channels[ch][idx]
            y = channels[ch][idx+1]
            z = channels[ch][idx+2]
            word = (x << 8) | y
            word = (word << 8) | z

            #Check for negatives
            negCheck = (word & 8388608)
            if (bin(negCheck)[2] == '1'):
                #You have a negative number
                word = word - (2**24)
            #do the math
            channelsV[ch].append(word*LSB)


if __name__ == "__main__":
    print("Input file name: ")
    raw_data = str(input())
    parse_accel_data(raw_data)
    parse_eeg(raw_data)
    print(len(channels[0])/3)
    count = 0
    for i in accel_data:
        count = count + 1
        print(i)
    print(count)