#!/usr/bin/env python3

import numpy
import matplotlib.pyplot as plt

t = numpy.linspace(0.1,10,100)

t = t + 0.1

o = numpy.log(t)
a = 1/(t*t)
b = 1/(numpy.log(t))
c = 1/(t)

#plt.plot(t, o, "r")
plt.plot(t, a, "r")
plt.plot(t, b, "g")
plt.plot(t, c, "b")
plt.show()