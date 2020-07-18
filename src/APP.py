import matplotlib.pyplot as plt
import math as Math
import numpy as np

path = []

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
        all_headings.extend([True if abs(head) > variance else False for head in headings])
        
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
radius = 1
step = .1
# 25 seems good, but u can play around with it if u want to :)
batch_size = 25
all_degrees = []


points = [(1, 1), (1, 4), (3, 5), (4, 3), (4, 1), (6, 1.1)] #for some reason y values cannot be the same
plt.plot(*zip(*points), '-o')

plt.plot(cur[0], cur[1], '-o')

for i in range(len(points) - 1):
        start = points[i]
        end = points[i+1]

        while (distance(cur[0], cur[1], end[0], end[1]) > radius):
                new_end = get_intersection(start, end, cur, radius)
                cur = get_intersection(cur, new_end, cur, step)
                degrees = get_degrees(new_end, cur)
                all_degrees.append(degrees)
                path.append(cur)
                #print("\n")
deviation = get_deviation(all_degrees, batch_size)
print(deviation)


plt.plot(*zip(*path), '-o')
plt.show()

