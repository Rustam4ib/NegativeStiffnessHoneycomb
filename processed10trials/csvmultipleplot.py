import pandas as pd
import matplotlib.pyplot as plt

#plt.rcParams["figure.figsize"] = [7.50, 3.50]
#plt.rcParams["figure.autolayout"] = True

# headers = ['fz', 'z']
# df5 = pd.read_csv('/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/rosbag_data/newpc_myrosbag_ur5_and_weiss_file_5try/new.csv', names=headers)
# df6 = pd.read_csv('/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/rosbag_data/newpc_myrosbag_ur5_and_weiss_file_6try/new.csv', names=headers)
# #print (df.dtypes)
# x5 = df5['z']
# y5 = df5['fz']
# y5 = pd.to_numeric(df5['fz'],errors = 'coerce')
# x5 = pd.to_numeric(df5['z'],errors = 'coerce')

# x6 = df6['z']
# y6 = df6['fz']
# y6 = pd.to_numeric(df5['fz'],errors = 'coerce')
# x6 = pd.to_numeric(df5['z'],errors = 'coerce')

# y5 = y5*(-1)
# x5 = x5.sort_values(ascending=True)
# x5 = x5*100
# y6 = y6*(-1)
# x6 = x6.sort_values(ascending=True)
# x6 = x6*100
# #print(x)

# plt.xlabel('position (cm)')
# plt.ylabel('force (N)')
# plt.plot(x5,y5)
# plt.plot(x6,y6)
# plt.grid(True)
# plt.show()

numFiles = 11 #Number of CSV files in your directory
separator = "," #Character that separates each value inside file
fExtension = ".csv" #Extension of the file storing the data

def MultiplePlots(xValues, allYValues):
    'Method to plot multiple times in one figure.'

    for yValues in allYValues:
        plt.plot(list(map(int, xValues)), list( map(float, yValues) ), label = "file" + str(i))

    plt.legend(loc = 'best')
    plt.show()
    return

def GetXandYValues(coordinates):
    'Method to get all coordinates from all CSV files.'
    xValues = []
    yValues = []
    allYValues = []
    fst = False
    for file in coordinates:
        for coordinate in file:
            if (fst == False):
                xValues.append(coordinate[0])
            yValues.append(coordinate[1])
        fst = True
        allYValues.append( yValues )
        yValues = []
    return xValues, allYValues

def GetCoordinates( n , separator , fExtension ):
    'Iterates through multiple CSV files and storing X values and Y values in different Lists'
    coordinates = [] #coordinates[0] = x values --- coordinates[1] = y values
    for i in range(n):
        coordinates.append( FillList( ReadFile("file" + str(i+1) + fExtension), separator ) )
    return coordinates

def ReadFile(path):
    'Function to read CSV file and store file data rows in list.'
    try:
        fileCSV = open(path,"r") #Opens file
        data = fileCSV.read() #Save file data in string
        listData = data.splitlines() #Split lines so you have List of all lines in file
        fileCSV.close() #Close file
    finally:
        return listData #Return list with file's rows

def FillList(myList, separator):
    'With this method you make a list containing every row from CSV file'
    valueTemp = ""
    listTemp = []
    newList = []
    for line in myList:
        for c in line:
            if c != separator:
                valueTemp += c
            else:
                listTemp.append( valueTemp )
                valueTemp = ""
        listTemp.append( valueTemp )
        newList.append(listTemp[:])
        valueTemp = ""
        del listTemp[:]
    return newList

xValues = GetXandYValues( GetCoordinates( numFiles, separator , fExtension) )[0]
allYValues = GetXandYValues( GetCoordinates( numFiles, separator , fExtension) )[1]

MultiplePlots( xValues, allYValues )