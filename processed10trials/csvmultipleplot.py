import pandas as pd
import matplotlib.pyplot as plt
import glob

path = "/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/processed10trials" # use your path
all_files = glob.glob(path + "/*.csv")

li = []
x = []
y = []

for filename in all_files:
    df = pd.read_csv(filename, index_col=None, header=0)
    x = df['z']
    y = df['fz']
    y = pd.to_numeric(df['fz'],errors = 'ignore')
    x = pd.to_numeric(df['z'],errors = 'ignore')
    y = y*(-1)
    x = x.sort_values(ascending=True)
    x = x*100
    plt.plot(x,y)
    #li.append(df)

# frame = pd.concat(li, axis=0, ignore_index=True)
# print(frame)
# x = frame['z']
# y = frame['fz']
# y = pd.to_numeric(df['fz'],errors = 'ignore')
# x = pd.to_numeric(df['z'],errors = 'ignore')
# y = y*(-1)
# x = x.sort_values(ascending=True)
# x = x*100
plt.xlabel('position (cm)')
plt.ylabel('force (N)')
# plt.plot(x,y)
plt.grid(True)
plt.show()
print("Success")