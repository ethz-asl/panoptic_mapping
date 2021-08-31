import numpy as np
from matplotlib import pyplot as plt
import sys


def p(f):
    return np.floor(f / 2**(np.floor(f / 128)))


def rollout(data):
    b = 0.0
    f = 0.0
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


def compute_rollouts():
    N = 500
    N_runs = 500

    result = np.zeros((N_runs, N))
    for i in range(1, N):
        for j in range(N_runs):
            data = np.random.randint(0, 2, i * 5)
            result[j, i] = (rollout(data) -
                            float(np.sum(data)) / len(data)) * 100.0

    print("Min: %.2f, Max: %.2f" % (np.min(result), np.max(result)))
    print("Mean: %.2f, Std: %.2f" %
          (np.mean(np.abs(result)), np.std(np.abs(result))))

    print(np.max(np.abs(result), axis=0))
    fig, axs = plt.subplots(3)
    axs[0].imshow(result, interpolation='none')
    axs[1].plot(np.max(np.abs(result), axis=0))
    axs[2].plot(np.mean(np.abs(result), axis=0))
    plt.show()


def compute_worst_case():
    N = 1000
    result = np.zeros((128, N))
    for i in range(1, N):
        # for j in range(128):
        result[:, i] = rollout([0] * i + [1] * i) * 100.0 - 50.0
    fig, axs = plt.subplots(3)
    axs[0].imshow(result, interpolation='none')
    axs[1].plot(result[0, :])

    plt.show()
    # result[i, j] = rollout([0])


# compute_rollouts()
compute_worst_case()