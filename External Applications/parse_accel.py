import csv

"""compute the 2's complement of int value val"""
def twos_comp(val, bits):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def parse_accel_data(fname):
    accel_data = list()

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

    # print(accel_data)

    with open('output.csv', 'w', newline='') as out_file:
        csv_writer = csv.writer(out_file, delimiter=',')
        csv_writer.writerow(['x', 'y', 'z'])
        for triple in accel_data:
            csv_writer.writerow(triple)

if __name__ == "__main__":
    print("Input file name: ")
    raw_data = str(input())
    parse_accel_data(raw_data)