# GNC is Graduated Non-Convexity, where a value will make function convex and le function become target equation.

import matplotlib.pyplot as plot
import numpy as np

def Equation(u, c, x):
  c2 = c^2
  x2 = np.square(x)
  return (u * c2 * x2)/(u * c2 + x2)
# main
r = 0
c = 1
m_range = 10
start = r - m_range;
end = r + m_range;
x = np.linspace(start, end, 100)

c2 = c^2
x2 = np.square(x)

y1 = Equation(1, c, x)
y2 = Equation(10, c, x)
y3 = Equation(100, c, x)

plot.plot(x, y1, label="y1")
plot.plot(x, y2, label="y2")
plot.plot(x, y3, label="y3")

plot.legend()

plot.show()