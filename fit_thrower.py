import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy import optimize


def func(x, a, b):
    return x*a + b


df = pd.read_csv('thrower_values.csv')
print(df)
plt.figure()
plt.plot(df['Dist'], df["Speed"])
plt.xlabel('Distance')
plt.ylabel('Speed')

popt, pcov = optimize.curve_fit(func, df['Dist'], df["Speed"])
print(popt)
plt.plot(df['Dist'], func(df['Dist'], *popt), 'r-')
plt.show()

print('Resulting func: thrower_speed =', popt[0], '* Distance +', popt[1])
