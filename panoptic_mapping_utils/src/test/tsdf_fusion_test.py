import numpy as np
import random
from matplotlib import pyplot as plt

# Params
n_measurements = 300
n_permutations = 10
truncation_d = 0.2
measurement_mode = "uniform"  # uniform,
w_max = 100

# Run
# Generate data
data = []
for t in range(n_measurements):
    w = 1
    d = 0.1
    # Sample the data.
    if measurement_mode == "uniform":
        w = random.uniform(0.1, 2)
        # d = random.uniform(-2*truncation_d, truncation_d)
        d = t / n_measurements
    data.append([d, w])

# Simulate order of integration.
distances = np.zeros((n_measurements, n_permutations))
weights = np.zeros((n_measurements, n_permutations))
for i in range(n_permutations):
    random.shuffle(data)

    # Integrate
    w = 0
    d = 0
    for t in range(n_measurements):
        d_new = min(max(data[t][0], -truncation_d), truncation_d)
        w_new = data[t][1]
        d = (d * w + d_new * w_new) / (w + w_new)
        w = min(w + w_new, w_max)
        distances[t, i] = d
        weights[t, i] = w

# Plot
fig, axes = plt.subplots(2, 2)
ax = axes[0, 0]
ax.plot(distances)
ax.set_ylabel("Distance")

ax = axes[1, 0]
ax.plot(weights)
ax.set_ylabel("Weight")

ax = axes[0, 1]
ax.plot(np.var(distances, 1))
ax.set_ylabel("Dist. Variance")

ax = axes[1, 1]
ax.plot(np.var(weights, 1))
ax.set_ylabel("Weight Variance")
plt.show()
