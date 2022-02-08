import pandas as pd
import matplotlib.pyplot as plt

#plt.rcParams["figure.figsize"] = [7.50, 3.50]
#plt.rcParams["figure.autolayout"] = True

headers = ['fz', 'z']
df = pd.read_csv('/home/rustam/catkin_ws/src/calibrate_rgb_sensor_rustam/src/rosbag_data/newpc_myrosbag_ur5_and_weiss_file_4try/new.csv', names=headers)
#print (df.dtypes)
x = df['z']
y = df['fz']
y = pd.to_numeric(df['fz'],errors = 'coerce')
x = pd.to_numeric(df['z'],errors = 'coerce')

y = y*(-1)
x = x.sort_values(ascending=True)
x = x*100
print(x)

plt.xlabel('position (cm)')
plt.ylabel('force (N)')
plt.plot(x,y)
plt.grid(True)
plt.show()
