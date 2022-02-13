from operator import le
from sqlite3 import Row
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np

mypath = "/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/processed10trials" # use your path
li_y = []
li_x = []
#The numericalSort() function splits out any digits in a filename, turns it into an actual number
numbers = re.compile("(\d+)")
def numericalSort(value):
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def graphdata(path):

    x = [62]
    y = [62]
    all_files = sorted(glob.glob(path + "/*.csv"), key=numericalSort)
    for filename in all_files:
        df = pd.read_csv(filename, index_col=False, header=0)
        #Set x axis
        x = df['z']
        x = pd.to_numeric(df['z'],errors = 'ignore')
        x = x.sort_values(ascending=True)
        #x = x[:-1]
        x = x*100

        x = x.loc[x>5.02]
        x = x.loc[x<6.9]

        x = x[0:61]
        #xnew = pd.Series(x)
        #Set y axis
        y = df['fz']
        y = pd.to_numeric(df['fz'],errors = 'ignore')
        y = y*(-1)
        y = y[0:61]
        li_x.append(x)
        li_y.append(y)
        plt.plot(x,y, alpha = 0.2)
        #print(x.shape)
        #print(y.shape)
        #mean_x = x.mean()
        #mean_y = y.mean()
        #print(mean_x)
        #print(mean_y)        
        #plt.plot(x,mean_y, alpha = 1.0)

    plt.xlabel('position (cm)')
    plt.ylabel('force (N)')
    plt.grid(True)
    #plt.show()

def processedData():
    row_num = []
    for i in range(len(li_x)):
            print("LEN:"+str(len(li_x[i])))
            #print(li_x[i])
            #row_num.append(len(li_x[i]))
    print("min:"+str(min(row_num)))        
    print("Success")



def mean_std():
    row_num = []
    mean_list = []
    mean = 0.000
    li_ynp = np.array(li_y)
    print("\n")
    print(li_y)
    print("\n")
    for j in range(61):
        for i in range(11):
            mean += li_ynp[i][j]
        mean /= 11    
        mean_list.append(mean)
    plt.plot(li_x[0],mean_list, alpha = 1.0)
    #plt.show()

def main():
    print("Hello World!")
    graphdata(mypath)
    #processedData()
    #showGraph()
    mean_std()
    plt.show()
    print("Bye World!")

if __name__ == "__main__":
    main()