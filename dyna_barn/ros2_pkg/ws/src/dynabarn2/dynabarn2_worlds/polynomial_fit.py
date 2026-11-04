"""Random polynomial trajectories from the original DynaBARN generator.

Ported from DynaBARN/polynomial_fit.py; the only change is that the speed
distribution uses a NumPy Gaussian instead of scipy.stats (same probabilities).
"""
import math
import random
import warnings
from random import choice

import numpy as np
from numpy.random import random_sample


def distance(start_location, end_location):
    return math.sqrt((start_location[0] - end_location[0]) ** 2
                     + (start_location[1] - end_location[1]) ** 2)


def gen_points(n):
    x_s = []
    y_s = []
    for _ in range(n):
        # change these numbers based on the start and end points
        x_s.append(random.randint(-10, 10))
        y_s.append(random.randint(-10, 10))
    return x_s, y_s


def _random_border_point():
    a, b = -10, 10
    r = (b - a) * random_sample() + a
    return choice([(choice([a, b]), r), (r, choice([a, b]))])


def get_random_traj_points(order):
    n = order
    x_s, y_s = gen_points(n + 1)

    # Fitting order n through n + 1 integer points is often ill-conditioned; that's expected here.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Polyfit may be poorly conditioned")
        p = np.poly1d(np.polyfit(x_s, y_s, n))
    xp = np.linspace(-10, 10, 10)
    pxp = p(xp)

    # find where y = 10 and -10
    x_10 = (p - 10).roots
    x_neg10 = (p - (-10)).roots
    # find where x = 10, -10
    y_10 = p(10)
    y_neg10 = p(-10)

    # keep only real crossings
    if not x_10.imag.any(0):
        x_s = np.concatenate((x_s, x_10.real), axis=None)
        for _ in x_10:
            y_s = np.append(y_s, 10)
    if not x_neg10.imag.any(0):
        x_s = np.concatenate((x_s, x_neg10.real), axis=None)
        for _ in x_10:
            y_s = np.append(y_s, -10)
    if not y_10.imag.any(0):
        y_s = np.concatenate((y_s, y_10), axis=None)
        x_s = np.append(x_s, 10)
    if not y_neg10.imag.any(0):
        y_s = np.concatenate((y_s, y_neg10), axis=None)
        x_s = np.append(x_s, -10)

    xp = np.append(xp, x_s)
    pxp = np.append(pxp, y_s)

    points = []
    for i in range(len(xp)):
        if -10 <= xp[i] <= 10 and -10 <= pxp[i] <= 10:
            points.append((xp[i], pxp[i]))

    seen = set()
    uniq = [x for x in points if x not in seen and not seen.add(x)]
    uniq = sorted(uniq, key=lambda x: x[0])

    # border points go at the beginning and the end of the sequence
    border_points = []
    normal_points = []
    for pt in uniq:
        if pt[0] in (10, -10) or pt[1] in (10, -10):
            border_points.append(pt)
        else:
            normal_points.append(pt)
    border_points = sorted(border_points, key=lambda x: x[0])
    while len(border_points) < 2:
        border_points.append(_random_border_point())
    start_border_point = border_points.pop(0)
    end_border_point = border_points.pop(-1)
    if not border_points:
        uniq = normal_points + border_points
    else:
        uniq = normal_points

    uniq = sorted(uniq, key=lambda x: x[0])
    uniq.insert(0, start_border_point)
    uniq.append(end_border_point)

    # reverse trajectory at random
    if random.choice([0, 1]) < .5:
        uniq.reverse()

    new_x = [pt[0] for pt in uniq]
    new_y = [pt[1] for pt in uniq]
    return new_x, new_y, uniq


def calc_time(x, y, points, avg_speed, min_speed, max_speed, std):
    speeds = np.arange(min_speed, max_speed, 0.1)
    # Speed distribution is the same for every segment, so compute it once
    if std > 0:
        prob = np.exp(-0.5 * ((speeds - avg_speed) / std) ** 2)
    else:  # constant speed: all mass on the speed closest to avg_speed
        prob = np.zeros(len(speeds))
        prob[np.argmin(np.abs(speeds - avg_speed))] = 1.0
    prob = prob / prob.sum()
    times = [0]
    for i in range(0, len(points) - 1):
        d = distance(points[i], points[i + 1])
        speed = 0
        while speed <= 0:
            speed = np.random.choice(speeds, 1, p=prob)
        times.append((d / speed + times[-1])[0])
    return times
