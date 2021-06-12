import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

import numpy as np
import pandas as pd

data = pd.read_csv('build/example.csv')


print("Dodo Alive\nEasy Viso Interface")
print("---------------------------------------------------------")

print("Available variables:\n", list(data.columns[:-1]))

inputX = input("Please select your x value: ") 
print("you selected for x:", inputX)
inputXLabel =  input("Please enter the axis label: ")
inputY = input("Please select your y value: ") 
print("you selected for y:", inputY)
inputYLabel =  input("Please enter the axis label: ")
print("---------------------------------------------------------")

#inputX = 'time'
#inputY = 'actualComX'
data.plot(x=inputX, y=inputY)

print("Generated plot:", inputXLabel, "(", inputX, ") over", inputYLabel, "(", inputY, ").")
plt.xlabel(inputXLabel)
plt.ylabel(inputYLabel)

plt.grid()
plt.show()
