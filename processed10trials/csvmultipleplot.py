from sqlite3 import Row
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re

mypath = "/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/processed10trials" # use your path

#The numericalSort() function splits out any digits in a filename, turns it into an actual number
numbers = re.compile("(\d+)")
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def graphdata(path):
    li = []
    x = []
    y = []
    all_files = sorted(glob.glob(path + "/*.csv"), key=numericalSort)
    for filename in all_files:
        df = pd.read_csv(filename, index_col=False, header=0)
        #Set x axis
        x = df['z']
        x = pd.to_numeric(df['z'],errors = 'ignore')
        x = x.sort_values(ascending=True)
        x = x*100
        #Set y axis
        y = df['fz']
        li.append(y)
        y = pd.to_numeric(df['fz'],errors = 'ignore')
        y = y*(-1)
    return x,y

def processedData():
    row_num = []
    for i in range(len(li)):
            print(len(li[i]))
            row_num.append(len(li[i]))
    print("min:"+str(min(row_num)))        
    print("Success")


def showGraph():
    x,y = graphdata(mypath)
    plt.plot(x,y)
    plt.xlabel('position (cm)')
    plt.ylabel('force (N)')
    plt.grid(True)
    plt.show()



def main():
    graphdata(mypath)
    #processedData()
    showGraph()
    print("Hello World!")

if __name__ == "__main__":
    main()