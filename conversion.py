import sys
import pandas as pd
import matplotlib.pyplot as plt

fileName = ""
# If user inputs a data file to use after the terminal input
if len(sys.argv) > 1:
    fileName = sys.argv[1]
else:
    # Default file to open if no other inputs are used
    fileName = r"G:\Shared drives\Case Rocket Team\2025 - 2026\26D - Measurements Subsystem\6. Data\03-03-2026testing0000.csv"

file = pd.read_csv(fileName, delimiter=",")

hasAccel = False

# Grab the header and delete time name
headerLine = file.columns.tolist()

if "accel_x" in headerLine:
    hasAccel = True
    headerLine = headerLine[1:-3]
else:
    headerLine = headerLine[1:]

# Number of Analog to Digital converters and number of gauges connected to each converter and number of cycles per adc
adcCount = 2
gauges = [0, 0]
cycles = [0, 0]

# To determine number of gauges and cycles per adc
for name in headerLine:
    
    # get the adc number and gauge number and cycle number
    adcIndex = int(name[3])
    gaugeNumber = int(name[9])
    cycleNumber = int(name[15])

    # If the gauge number is greater than our counter then we add one
    if gaugeNumber + 1 > gauges[adcIndex]:
        gauges[adcIndex] += 1

    if cycleNumber + 1 > cycles[adcIndex]:
        cycles[adcIndex] += 1

# get total number of data measurements
numGaugeMeasurements = gauges[0] * cycles[0] + gauges[1] * cycles[1]

# get all gauge values
gaugeVoltagesData = []
for measurementNum in range(numGaugeMeasurements):

    # get gauge data and convert to voltage
    gaugeVoltagesData.append([value * 5 / (2**16) for value in file[headerLine[measurementNum]].values.tolist()])

# get all time values
measurementTimes = file["time"].values.tolist()

# gauge measurement times for each gauge (by index) and the changes in measurement time for each measurement
allTimeChanges = []
allGaugeMeasurementTimes = []

# calculate time step for each measurement
for i in range(1, len(measurementTimes)):
    timeChange = (measurementTimes[i] - measurementTimes[i - 1]) / numGaugeMeasurements
    allTimeChanges.append(timeChange)

# output all combined gauge voltages here:
gaugeVoltages = []

# combine data columns for the same gauges and calculate time for each measurement
for adcIndex in range(adcCount):

    cycleCount = cycles[adcIndex]
    # if the adc has more than one cycle
    if cycleCount > 1:

        for gaugeIndex in range(gauges[adcIndex]):

            # collect and combine all cycles for gauges
            singleGaugeVoltages = []
            gaugeMeasurementTimes = []
            # get all the cycles for a gauge
            for cycleIndex in range(cycleCount):
                numPrevColumns = 0
                if adcIndex > 0:
                    numPrevColumns = gauges[adcIndex-1] * cycles[adcIndex-1]
                headerIndex = numPrevColumns + cycleIndex * gauges[adcIndex] + gaugeIndex

                # add the column data to the list of columns for a single gauge
                singleGaugeVoltages.append(gaugeVoltagesData[headerIndex])

                # Calculate time when measurements were taken for a gauge at each cycle
                timeOffset = [(adcIndex + 2 * gaugeIndex + gauges[adcIndex] * cycleIndex * 2) * timeChange for timeChange in allTimeChanges]
                
                # add it to the initial measurement time
                measurementTime = [a + b - measurementTimes[0] for a, b in zip(measurementTimes, timeOffset)]
                gaugeMeasurementTimes.append(measurementTime)

            # zipper the lists together to form one long one
            singleGaugeVoltages = [value for sublist in zip(*singleGaugeVoltages) for value in sublist]
            singleGaugeMeasurementTimes = [value for sublist in zip(*gaugeMeasurementTimes) for value in sublist]

            # add the combined gauge data to the gauge voltages
            gaugeVoltages.append(singleGaugeVoltages)
            allGaugeMeasurementTimes.append(singleGaugeMeasurementTimes)

    # only one cycle
    else:
        for gaugeIndex in range(gauges[adcIndex]):
            numPrevColumns = 0
            if adcIndex > 0:
                numPrevColumns = gauges[adcIndex-1] * cycles[adcIndex-1]
            headerIndex = numPrevColumns + gaugeIndex
            # add the column data to the list of columns for a single gauge
            gaugeVoltages.append(gaugeVoltagesData[headerIndex])

            # Calculate time when measurements were taken for a gauge at each cycle
            timeOffset = [(adcIndex + 2 * gaugeIndex) * timeChange for timeChange in allTimeChanges]
            
            # add it to the initial measurement time
            measurementTime = [a + b - measurementTimes[0] for a, b in zip(measurementTimes, timeOffset)]

            allGaugeMeasurementTimes.append(measurementTime)

total_time_diff = 0
for t0, t1 in zip(measurementTimes[:-1], measurementTimes[1:-2]):
    total_time_diff += t1 - t0

avg_time_diff = total_time_diff / (len(measurementTimes) - 1)
sampling_rate = 1.0 / (avg_time_diff * 1e-6)

print(f"Average Time Diff  - {avg_time_diff:.2f} us")
print(f"Sampling Rate      - {sampling_rate:.2f} Hz")
print(f"ADC0 Sampling Rate - {cycles[0] * sampling_rate:.2f} Hz")
print(f"ADC1 Sampling Rate - {cycles[1] * sampling_rate:.2f} Hz")

fig, axs = plt.subplots(2 if hasAccel else 1, 1)

if hasAccel:
    accelTimes = [t - measurementTimes[0] for t in measurementTimes]
    accelX = [x * (8 / 2 ** 15) for x in file["accel_x"].values.tolist()]
    accelY = [y * (8 / 2 ** 15) for y in file["accel_y"].values.tolist()]
    accelZ = [z * (8 / 2 ** 15) for z in file["accel_z"].values.tolist()]


    axs[1].plot(accelTimes, accelX, label="Accel X")
    axs[1].plot(accelTimes, accelY, label="Accel Y")
    axs[1].plot(accelTimes, accelZ, label="Accel Z")

    axs[1].set_xlabel("Time")
    axs[1].set_ylabel("Acceleration (g)")
    axs[1].legend()

# Trim voltage data to match the length of the time data
for i in range(len(allGaugeMeasurementTimes)):
    if len(allGaugeMeasurementTimes[i]) < len(gaugeVoltages[i]):
        gaugeVoltages[i] = gaugeVoltages[i][:len(allGaugeMeasurementTimes[i])]

gaugeAxes = axs[0] if hasAccel else axs

# plot data
for i in range(len(gauges)):
    for j in range(gauges[i]):
        index = gauges[i] * i + j
        gaugeAxes.plot(allGaugeMeasurementTimes[index], gaugeVoltages[index], label = f'ADC{i+1} Gauge{j+1}', marker='o', linestyle='-', markersize=1.5, color=f'C{index}')
    
gaugeAxes.set_xlabel("Time")
gaugeAxes.set_ylabel("Voltage (V)")
gaugeAxes.legend()

plt.show()


