import sys
import pandas as pd
import matplotlib.pyplot as plt

fileName = ""
# If user inputs a data file to use after the terminal input
if len(sys.argv) > 1:
    fileName = sys.argv[1]
else:
    # Default file to open if no other inputs are used
    fileName = r"G:\Shared drives\Case Rocket Team\2025 - 2026\26D - Measurements Subsystem\6. Data\02-28-2026-testing0001.csv"

file = pd.read_csv(fileName, delimiter=",")

# Grab the header and delete time name
headerLine = file.columns.tolist()
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

# get all time values
time = file['time'].values.tolist();

# get total number of data measurements
datas = gauges[0] * cycles[0] + gauges[1] * cycles[1]


# get all gauge values
gaugeVoltagesData = []
for i in range(datas):
    # get gauge data and convert to voltage
    gaugeVoltagesData.append([i * 5 / (2**16) for i in file[headerLine[i]].values.tolist()])

# get all time values
measurementTimes = file["time"].values.tolist()

# gauge measurement times for each gauge (by index) and the changes in measurement time for each measurement
allTimeChanges = []
allGaugeMeasurementTimes = []

# calculate time step for each measurement
for i in range(1, len(measurementTimes)):
    timeChange = (measurementTimes[i] - measurementTimes[i - 1]) / datas
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
            for index in range(cycleCount):

                headerIndex = adcIndex * gauges[adcIndex] * cycles[adcIndex] + index * gauges[adcIndex]

                # add the column data to the list of columns for a single gauge
                singleGaugeVoltages.append(gaugeVoltagesData[headerIndex])

                # Calculate time when measurements were taken for a gauge at each cycle
                timeOffset = [(adcIndex + 2 * gaugeIndex + gauges[adcIndex] * index * 2) * i for i in allTimeChanges]
                
                # add it to the initial measurement time
                measurementTime = [a + b - measurementTimes[0] for a, b in zip(measurementTimes, timeOffset)]
                gaugeMeasurementTimes.append(measurementTime)

            # zipper the lists together to form one long one
            singleGaugeVoltages = [i for sublist in zip(*singleGaugeVoltages) for i in sublist]
            singleGaugeMeasurementTimes = [i for sublist in zip(*gaugeMeasurementTimes) for i in sublist]

            # add the combined gauge data to the gauge voltages
            gaugeVoltages.append(singleGaugeVoltages)
            allGaugeMeasurementTimes.append(singleGaugeMeasurementTimes)

    # only one cycle
    else:
        for gaugeIndex in range(gauges[adcIndex]):
            headerIndex = adcIndex * gauges[adcIndex] * cycles[adcIndex] + gaugeIndex

            # add gauge data even if only one cycle to have each correspond to their indices
            gaugeVoltages.append(gaugeVoltagesData[headerIndex])

            timeOffset = [(adcIndex + 2 * gaugeIndex) * i for i in allTimeChanges]
            
            # add it to the initial measurement time
            measurementTime = [a + b - measurementTimes[0] for a, b in zip(measurementTimes, timeOffset)]

            allGaugeMeasurementTimes.append(measurementTime)

# Trim voltage data to match the length of the time data
for i in range(len(allGaugeMeasurementTimes)):
    if len(allGaugeMeasurementTimes[i]) < len(gaugeVoltages[i]):
        gaugeVoltages[i] = gaugeVoltages[i][:len(allGaugeMeasurementTimes[i])]

# plot data
for i in range(len(allGaugeMeasurementTimes)):
    plt.plot(allGaugeMeasurementTimes[i], gaugeVoltages[i], label = f'Gauge{i+1}', marker='o', linestyle='-', markersize=1.5, color=f'C{i}')
    
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()
plt.show()