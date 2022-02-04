import matplotlib.pyplot as plt
import numpy as np

import scipy.stats
import numpy as np
import matplotlib.pyplot as plt


def random_line(m, b, sigma, size):
    xdata = np.linspace(-10, 10, size)

    # Generate normally distributed random error ~ N(0, sigma**2)
    errors = scipy.stats.norm.rvs(loc=0, scale=sigma, size=size)
    ydata = m * xdata + b + errors

    clean_xdata = np.linspace(-10, 10, size/3)
    clean_ydata = m * clean_xdata + b
    ii = np.searchsorted(xdata, clean_xdata)
    xdata = np.insert(xdata, ii, clean_xdata)
    ydata = np.insert(ydata, ii, clean_ydata)

    return xdata, ydata


def ransac(xdata, ydata, k, t, d):
    iterations = 0
    inliers = []
    model = None
    fp1 = None
    fp2 = None

    while iterations < k:
        maybe_inliers = np.random.randint(0, len(xdata)-1, 2)
        also_inliers = []

        p1 = np.array([xdata[maybe_inliers[0]], ydata[maybe_inliers[0]]])
        p2 = np.array([xdata[maybe_inliers[1]], ydata[maybe_inliers[1]]])
        maybe_model = p2-p1
        best_err = np.inf

        for x, y in zip(xdata, ydata):
            if (x == p1[0] and y == p1[1]) or (x == p2[0] and y == p2[1]):
                continue

            p3 = np.array([x, y])
            dist = np.linalg.norm(np.cross(maybe_model, p1-p3)) / np.linalg.norm(maybe_model)
            if dist < t:
                also_inliers.append(p3)

        if len(also_inliers) > len(inliers):
            model = maybe_model
            inliers = also_inliers
            fp1 = p1
            fp2 = p2

        if len(inliers) >= d:
            print("best model after", iterations,"iterations: ", fp1, fp2, "with", len(inliers), "of", d)
            return model, np.array(inliers), fp1, fp2

        iterations += 1

    print("best model after", iterations,"iterations: ", fp1, fp2, "with", len(inliers), "of", d)
    return model, np.array(inliers), fp1, fp2


if __name__ == "__main__":
    size = 100
    xs, ys = random_line(2.0, 3.0, 10.0, size=size)
    print("Generated", len(xs), "points")

    # max iterations
    k = 100
    # threshold value to determine data points that are fir well by model
    t = 0.5
    # number of close data points required to assert that a model fits well to data
    d = int(size * 0.6)

    print("ransac params: k {}, t {}, d {}".format(k, t, d))

    model, inliers, fp1, fp2 = ransac(xs, ys, k, t, d)

    plt.grid(which='major', color='#DDDDDD', linewidth=0.8)
    plt.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.5)
    plt.minorticks_on()

    magnitude = np.sqrt(model[0]**2+model[1]**2)
    plt.scatter(xs, ys)
    plt.scatter(fp1[0], fp1[1], color='red', s=70, marker='o')
    plt.scatter(fp2[0], fp2[1], color='red', s=70, marker='o')
    plt.scatter(inliers[:,0], inliers[:,1], color='lightgreen')
    plt.show()