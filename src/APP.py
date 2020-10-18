import matplotlib.pyplot as plt
import math as Math
import numpy as np
from itertools import islice

def point_line_distance (p1, p2, p3):

  dy = p1[1]-p2[1]
  dx = p1[0]-p2[0]

  if (dy == 0):
    return abs(p1[1] - p3[1])
  if (dx == 0):
    return abs(p1[0] - p3[0])

  slope = dy/dx
  a = - slope
  b = 1
  c = slope * p2[0] - p2[1]

  return abs((a * p3[0] + b * p3[1] + c) / Math.sqrt(a ** 2 + b ** 2))

def get_deviation(headings, numb):

    headingsLength = len(headings)
    prev_heading = 0
    all_headings = []

    for index in range(numb, headingsLength, numb):
        delta_numb = headingsLength-index
        if delta_numb < numb:
            index+=delta_numb

        heading_range = headings[prev_heading:index]
        heading_range_length = len(heading_range)
        prev_heading=index

        mean = sum(heading_range)/heading_range_length
        varianceList = [abs(heading-mean)**2 for heading in heading_range]
        variance = Math.sqrt(sum(varianceList)/heading_range_length)
        #   True = Turning | False = Straight

        all_headings.extend([variance])

    return all_headings


def distance (x1, y1, x2, y2):

    return Math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_degrees(point1, point2):

    slope=(point2[1]-point1[1])/(point2[0]-point1[0])
    return -Math.degrees(Math.atan(slope))


def get_intersection(start, end, cur, radius):

        p1 = (start[0]-cur[0], start[1]-cur[1])
        p2 = (end[0]-cur[0], end[1]-cur[1])
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        d = Math.sqrt(dx * dx + dy * dy)
        D = p1[0] * p2[1] - p2[0] * p1[1]


        discriminant = abs(radius ** 2 * d ** 2 - D ** 2)


        x1 = (D * dy + np.sign(dy) * dx * Math.sqrt(discriminant)) / (d ** 2);
        y1 = (-D * dx + abs(dy) * Math.sqrt(discriminant)) / (d ** 2);
        x2 = (D * dy - np.sign(dy) * dx * Math.sqrt(discriminant)) / (d ** 2);
        y2 = (-D * dx - abs(dy) * Math.sqrt(discriminant)) / (d ** 2);

        distance1 = distance(p2[0], p2[1], x1, y1)
        distance2 = distance(p2[0], p2[1], x2, y2)

        if (distance1 < distance2): return (x1+cur[0], y1+cur[1])
        elif (distance1 > distance2): return (x2+cur[0], y2+cur[1])

cur = (.2, .2)
radius = 300
step = 10
batch_size = 10
all_degrees = []
points = [(0, 0), (800, 0.1), (800, 750), (220, 1100)]
stop_points = [(430, 0), (900, 400), (480, 880)]

path = []
PID = []
no_PID = []

for i in range(len(points) - 1):
        start = points[i]
        end = points[i+1]

        while (distance(cur[0], cur[1], end[0], end[1]) > radius):

            new_end = get_intersection(start, end, cur, radius)
            cur = get_intersection(cur, new_end, cur, step)
            degrees = get_degrees(new_end, cur)
            all_degrees.append(degrees)
            path.append(cur)

            if points[-1] != end:
                no_PID.append(cur)
            else:
                PID.append(cur)

deviation = get_deviation(all_degrees, batch_size)

def show_points(points):

    end_point = points[-1]
    plt.plot(*zip(*points), '-o')
    plt.plot(cur[0], cur[1], '-o')
    plt.scatter(*zip(*stop_points), s=150, color='none', edgecolor='red')


def show_PID(no_PID, PID):

    show_points(points)
    plt.plot(*zip(*no_PID), '-o')
    plt.plot(*zip(*PID), '-o')
    plt.show()


def show_std(deviation, path):

    show_points(points)
    segment_length = int(len(path)/len(deviation))
    leftover = len(path)%len(deviation)
    split = [segment_length] * (len(deviation) - 1) + [segment_length + leftover]
    Inputt = iter(path)
    Output = [list(islice(Inputt, elem)) for elem in split]
    for x in Output:
        plt.plot(*zip(*x), '-o')

    plt.show()

#show_std(deviation, path)

show_PID(no_PID, PID)

#print(deviation)

#print(get_degrees((0, 0), (1000, 1000)))
