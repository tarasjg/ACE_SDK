import csv
import matplotlib.pyplot as plt
 
#Time vector, 3 different axes on the accel., 8 channels on the helmet
time = []
ch1 = []
ch2 = []
ch3 = []
ch4 = []
ch5 = []
ch6 = []
ch7 = []
ch8 = []
x = []
y = []
z = []

with open('output.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    lineCount = 0
    for row in csv_reader:
        #Ignore the headers if there are headers
        if lineCount == 0:
            lineCount += 1
        elif(lineCount != 0):
            #Fill out the vectors
                time.append(float(row[0]))
                ch1.append(float(row[1]))
                ch2.append(float(row[2]))
                ch3.append(float(row[3]))
                ch4.append(float(row[4]))
                ch5.append(float(row[5]))
                ch6.append(float(row[6]))
                ch7.append(float(row[7]))
                ch8.append(float(row[8]))
                x.append(float(row[9]))
                y.append(float(row[10]))
                z.append(float(row[11]))
        lineCount += 1

#Plotting Channels
plot1, axs = plt.subplots(8,1)

#Channel 1
axs[0].plot(time,ch1)
axs[0].set_title('CH1 vs Timestamp')
axs[0].set(xlabel = 'Timestamp', ylabel = 'CH1 (V)')

#Channel 2
axs[1].plot(time,ch2)
axs[1].set_title('CH2 vs Timestamp')
axs[1].set(xlabel = 'Timestamp', ylabel = 'CH2 (V)')

#Channel 3
axs[2].plot(time,ch3)
axs[2].set_title('CH3 vs Timestamp')
axs[2].set(xlabel = 'Timestamp', ylabel = 'CH3 (V)')

#Channel 4
axs[3].plot(time,ch4)
axs[3].set_title('CH4 vs Timestamp')
axs[3].set(xlabel = 'Timestamp', ylabel = 'CH4 (V)')


#Channel 5
axs[4].plot(time,ch5)
axs[4].set_title('CH5 vs Timestamp')
axs[4].set(xlabel = 'Timestamp', ylabel = 'CH5 (V)')

#Channel 6
axs[5].plot(time,ch6)
axs[5].set_title('CH6 vs Timestamp')
axs[5].set(xlabel = 'Timestamp', ylabel = 'CH6 (V)')

#Channel 7
axs[6].plot(time,ch7)
axs[6].set_title('CH7 vs Timestamp')
axs[6].set(xlabel = 'Timestamp', ylabel = 'CH7 (V)')

#Channel 8
axs[7].plot(time,ch8)
axs[7].set_title('CH8 vs Timestamp')
axs[7].set(xlabel = 'Timestamp', ylabel = 'CH8 (V)')

#Plotting Channels
plot2, axs2 = plt.subplots(3,1)

axs2[0].plot(time,x)
axs2[0].set_title('X vs Timestamp')
axs2[0].set(xlabel = 'Timestamp', ylabel = 'X')

axs2[1].plot(time,y)
axs2[1].set_title('Y vs Timestamp')
axs2[1].set(xlabel = 'Timestamp', ylabel = 'Y')

axs2[2].plot(time,z)
axs2[2].set_title('Z vs Timestamp')
axs2[2].set(xlabel = 'Timestamp', ylabel = 'Z')

plot1.tight_layout()
plot2.tight_layout()

plt.show()

'''
#Plotting Channels
plot1, axs = plt.subplots(1,1)

#Channel 1
axs.plot(time,ch1)
axs.set_title('CH1 vs Timestamp')
axs.set(xlabel = 'Timestamp', ylabel = 'CH1 (V)')
xy = (2.2e4,-.045)
axs.annotate('Max:' + str(max(ch1)),xy)
xy = (2.2e4,-.05)
axs.annotate('Min:' + str(min(ch1)),xy)
plot1.tight_layout()
plt.show()
'''