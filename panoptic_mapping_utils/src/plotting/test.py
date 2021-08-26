import numpy as np
from matplotlib import pyplot as plt
import sys


def p(f):
    return np.floor(f / 2**(np.floor(f / 128)))


def rollout(data):
    b = 0
    f = 0
    count = 0
    for x in data:
        count += 1
        if x == 1:
            b += 1
        else:
            f += 1
        # if f >= 256 or b >= 256:
        if count > 128:
            count = 0
            f = np.floor(f / 2)
            b = np.floor(b / 2)
    return b / (b + f)


N = 100
N_runs = 100

result = np.zeros((N_runs, N))
for i in range(1, N):
    # data = np.array([1] * i + [0] * (N-i))
    for j in range(N_runs):
        # np.random.shuffle(data)
        data = np.random.randint(0, 2, i * 128)
        result[j, i] = (rollout(data) - np.sum(data) / i / 128) * 100.0
    # result[N_runs:,i] = np.mean(result[:N_runs,i])

print("Min: %.2f, Max: %.2f" % (np.min(result), np.max(result)))
print("Mean: %.2f, Std: %.2f" %
      (np.mean(np.abs(result)), np.std(np.abs(result))))

# print(result[-1,:])
print(np.max(np.abs(result), axis=0))
# plt.subplot(0,0)
fig, axs = plt.subplots(3)
axs[0].imshow(result, interpolation='none')
# plt.subplot(0,1)
axs[1].plot(np.max(np.abs(result), axis=0))
axs[2].plot(np.mean(np.abs(result), axis=0))
plt.show()

# counts = [p(f) for f in range(N)]
# prob_est = np.zeros((N,N))
# prob_true = np.zeros((N,N))

# for i in range(N):
#   for j in range(N):
#     if i == 0 and j == 0:
#       prob_est[i,j] = 0
#       prob_true[i,j] = 0
#     else:
#       prob_est[i,j] = counts[i] / (counts[i] + counts[j])
#       prob_true[i,j] = float(i) / (i+j)

# diff = (prob_est-prob_true) * 100.0
# print("Min: %.2f, Max: %.2f" % (np.min(diff), np.max(diff)))

# plt.imshow(diff, interpolation='none')
# plt.show()
