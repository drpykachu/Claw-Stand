



import matplotlib.pyplot as plt
import numpy as np

points = 201

x = np.linspace(-100,100,points)
x2 = x
y = (x)**2
y2 = np.ones(points)*np.max(y)
plt.plot(list(x)+list(x2),list(y)+list(y2),'.')

#use this function - bs
x = np.linspace(-100,100,points)
x2 = x
y = (x*1.5)**2
y2 = np.ones(points)*np.max(y)
plt.plot(list(x)+list(x2),list(y)+list(y2),'.')


plt.show()