import numpy as np
from matplotlib import pyplot as plt
import sys
from matplotlib import cm

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

# Make data.
MAX_WEIGHT = 10000.0
THRESH = 10000.0
X = np.linspace(0, MAX_WEIGHT)
Y = np.linspace(0, MAX_WEIGHT)
X, Y = np.meshgrid(X, Y)
Z = np.sqrt(
    np.minimum(X / THRESH, np.ones_like(X)) *
    np.minimum(Y / THRESH, np.ones_like(Y)))

for i in range(1, 10):
    for j in range(i, 10):
        print("x: %.2f, y: %.2f, z: %.2f" %
              (X[i * 5, j * 5] / THRESH, Y[i * 5, j * 5] / THRESH, Z[i * 5,
                                                                     j * 5]))

# Plot the surface.
surf = ax.plot_surface(X,
                       Y,
                       Z,
                       cmap=cm.coolwarm,
                       linewidth=0,
                       antialiased=False)

# Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
# ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
# ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
