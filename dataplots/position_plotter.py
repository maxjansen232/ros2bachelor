
import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv("/home/lodrik/ros2_wss/bagfiles/pid_tuner_aggro_arctan.csv")

print(data.head())

df = data.drop(data.columns[7:], axis=1)
df = df.drop(df.columns[[2,3]], axis=1)

new_columns = ["sekunder","nanosekunder","X","Y","Z"]
df.columns = new_columns

print(df.head())

df["nanosekunder"] = df["nanosekunder"]*(10**-9)

df["time"] = df["sekunder"]+df["nanosekunder"]
df['X'] = df['X']*1000
df['Y'] = df['Y']*1000
df['Z'] = df['Z']*1000
print(df.head())


pltx = df.plot(x='time', y='X', title = "X-bevegelse")
pltx.axhline(y=20, color='red', linestyle='--', linewidth=2)
pltx.axhline(y=-20, color='red', linestyle='--', linewidth=2)
pltx.axhline(y=0.00, color='green', linestyle='--', linewidth=2)
pltx.grid()

plty = df.plot(x='time', y='Y', title='y-bevegelse')
plty.axhline(y=20, color='red', linestyle='--', linewidth=2)
plty.axhline(y=-20, color='red', linestyle='--', linewidth=2)
plty.axhline(y=-0.00, color='green', linestyle='--', linewidth=2)
plty.grid()

pltz = df.plot(x='time', y='Z', title='z-bevegelse')
#plty.axhline(y=20, color='red', linestyle='--', linewidth=2)
#plty.axhline(y=-20, color='red', linestyle='--', linewidth=2)
pltz.axhline(y=25.00, color='green', linestyle='--', linewidth=2)
pltz.grid()

#SENSOR DATA PLOTS
sd = pd.read_csv("/home/lodrik/ros2_wss/bagfiles/nearlycompletesensordata2.csv")
print(sd.head())
sd.columns = ['a', 'b', 'c', 'z', 'time']
sd['x_error'] = (sd['c'] -0.5*(sd['a'] + sd['b'])) /(sd['c']+0.5*(sd['a'] + sd['b']))
sd['y_error'] = (sd['a']-1.325*sd['b'])/(sd['a'] + 1.325*sd['b'])
sd['z_error'] = (30000-sd['z'])/(30000)

xsplot = sd.plot(x='time', y='x_error', title='Error x')
xsplot.axhline(y=-0.00, color='green', linestyle='--', linewidth=2)

ysplot = sd.plot(x='time', y='y_error', title='Error y')
ysplot.axhline(y=-0.00, color='green', linestyle='--', linewidth=2)

zsplot = sd.plot(x='time', y='z_error', title='Error z')
zsplot.axhline(y=0.00, color='green', linestyle='--', linewidth=2)


plt.show()

#df.to_csv("PATH")