#!/usr/bin/env python3

import unittest
from math import sqrt, sin

### tuning variables ###

position_delta = 2. # how many meters to move before appending a new position to return_path
rdp_epsilon = position_delta * 1/4
cleanup_length = 10 # The number of points stored in memory that triggers the cleanup method
max_path_len = 50

def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def hypot3(p1, p2):
    return sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2 )

'''
    Returns the closest distance between two line segments in 3D space,
    and returns the halfway point between along the shortest line segment between the given input line segments.
    The first line segment runs from point p1 to p2, the second through p3 and p4.
'''
def segment_segment_dist(p1, p2, p3, p4):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    x4, y4, z4 = p4

    u = (x2-x1, y2-y1, z2-z1) # slope of line 1
    v = (x4-x3, y4-y3, z4-z3) # slope of line 2
    w = (x1-x3, y1-y3, z1-z3) # vector between line 1 and 2

    a = dot_product(u,u)
    b = dot_product(u,v)
    c = dot_product(v,v)
    d = dot_product(u,w)
    e = dot_product(v,w)
    D = a*c-b*b

    t1, t2 = 0,0 # the parameter for the position on line1 and line2 which define the closest points.

    if(D < 0.0000001): # almost parallel. This avoid division by 0.
        p1p3 = sqrt((x3-x1)**2+(y3-y1)**2+(z3-z1)**2), ( (x3-x1)/2, (y3-y1)/2, (z3-z1)/2 )
        p1p4 = sqrt((x4-x1)**2+(y4-y1)**2+(z4-z1)**2), ( (x4-x1)/2, (y4-y1)/2, (z4-z1)/2 )
        p2p3 = sqrt((x3-x2)**2+(y3-y2)**2+(z3-z2)**2), ( (x3-x2)/2, (y3-y2)/2, (z3-z2)/2 )
        p2p4 = sqrt((x4-x2)**2+(y4-y2)**2+(z4-z2)**2), ( (x4-x2)/2, (y4-y2)/2, (z4-z2)/2 )
        return min([p1p3, p1p4, p2p3, p2p4], key=lambda p: p[0])
    else:
        t1 = (b*e-c*d)/D
        t2 = (a*e - b*d)/D

        # restrict both parameters between 0 and 1.
        t1 = min(max(0,t1),1)
        t2 = min(max(0,t2),1)

        # difference between two closest points
        dP = (w[0]+t1*u[0]-t2*v[0], w[1]+t1*u[1]-t2*v[1], w[2]+t1*u[2]-t2*v[2])
        halfway_point = ( (x1+t1*u[0]+x3+t2*v[0])/2, (y1+t1*u[1] + y3+t2*v[1])/2, (z1+t1*u[2]+z3+t2*v[2])/2)

        return sqrt(dot_product(dP, dP)), halfway_point

'''
    Returns the closest distance from a point to a 3D line. The line is defined by any 2 points
    see https://stackoverflow.com/questions/1616050/minimum-perpendicular-distance-of-a-point-to-a-line-in-3d-plane-algorithm
'''
def point_line_dist(point, line):
    # triangle side lengths
    a = hypot3(point,line[0])
    b = hypot3(line[0],line[1])
    c = hypot3(line[1],point)

    s = (a+b+c)/2. # semiperimeter of triangle

    area = sqrt(s*(s-a)*(s-b)*(s-c))

    return 2*area/b

# TODO redo this iteratively, not recursively, then ignore any 2 points before clean_pos for simplification
def rdp(path, epsilon):
    max_index = -1
    max_dist = 0

    for i in range(1, len(path)-1):
        dist = point_line_dist( path[i], (path[0],path[-1]) )
        if dist > max_dist:
            max_index = i
            max_dist = dist
    if max_dist > epsilon:
        l1 = rdp(path[:max_index+1], epsilon)
        l2 = rdp(path[max_index:], epsilon)
        path = l1[:-1] + l2
    else:
        path = [path[0], path[-1]]
    return path

'''
    Takes a path and runs 2 cleanup steps: pruning, then simplification.

    The pruning step defines line segments from point 1 to 2, point 2 to 3, ...
    Then it compares (almost) all line segments to see how close they got, in 3D space.
    If they got close enough, defined by the parameter 'position_delta', all path points
    between those two line segments are deleted, and replaced by a single point halfway between
    where the two previous line segments were closest.

    This algorithm will never compare two consecutive line segments. Obviously
    the segments (p1,p2) and (p2,p3) will get very close (they touch), but there would be nothing to trim between them.

    If the deletion is triggered, the pruning step is complete. Since certain line segments are now gone,
    it does not make sense to keep comparing using those (potentially deleted) line segments. The goal of this algorithm
    is not to find the optimal simplified path, but rather to simplify it enough that it is not at risk of running out of memory.

    The simplification step uses the Ramer-Douglas-Peucker algorithm. See Wikipedia for description.

    clean_pos tells us the position up to which the path has already been cleaned. There's no point in doing any simplification/pruning
    among the first few points which have already been simplified and pruned.

'''
class Path:
    def __init__(self, path):
        self.path = path
        self.clean_pos = 0

    def append_if_far_enough(self, p):
        if len(self.path) >= 50:
            raise Exception("Out of Memory. Safe RTL unavailabe.")

        x,y,z = p
        x_old, y_old, z_old = self.path[-1]
        if (x-x_old)**2+(y-y_old)**2+(z-z_old)**2 >= position_delta**2:
            self.path.append(p)

    def get(self, i):
        return self.path[i]

    def cleanup(self):
        # if the path is short, don't bother. TODO improve this
        if len(self.path) < cleanup_length:
            return

        # pruning step
        pruning_occured = False
        for i in range(1, len(self.path)-1):
            for j in range(max(self.clean_pos,i)+2, len(self.path)-1):
                dist = segment_segment_dist(self.path[i], self.path[i+1], self.path[j], self.path[j+1])
                if dist[0] <= position_delta:
                    self.path = self.path[:i+1] + [dist[1]] + self.path[j+1:]
                    pruning_occured = True
                    self.clean_pos = min(self.clean_pos, i)
                    break
            else: # break out of both for loops
                continue
            break


        # simplification step. Uses Ramer-Douglas-Peucker algorithm
        self.path = rdp(self.path, epsilon = rdp_epsilon)
        self.clean_pos = len(self.path)-1

class TestLineCalculations(unittest.TestCase):
    def test_perpendicular(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (0,0,1)
        p4 = (0,1,1)
        self.assertEqual((1,(0,0,0.5)), segment_segment_dist(p1,p2,p3,p4))

    def test_parallel(self):
        p1 = (0,0,0)
        p2 = (1,1,0)
        p3 = (0,0,1)
        p4 = (1,1,1)
        self.assertEqual((1, (0.0, 0.0, 0.5)), segment_segment_dist(p1,p2,p3,p4))

    def test_intersecting(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (0,0,0)
        p4 = (0,1,0)
        self.assertEqual((0,(0,0,0)), segment_segment_dist(p1,p2,p3,p4))

    def test_identical(self):
        p1 = (0,0,0)
        p2 = (1,1,0)
        self.assertEqual((0,(0,0,0)), segment_segment_dist(p1,p2,p1,p2))
        self.assertEqual((0,(0,0,0)), segment_segment_dist(p1,p2,p2,p1))

    def test_parallel_but_spaced_out(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (3,0,0)
        p4 = (4,0,0)
        self.assertEqual((2, (1.0, 0.0, 0.0)), segment_segment_dist(p1,p2,p3,p4))

    def test_perpendicular_but_spaced_out(self):
        p1 = (-2,0,0)
        p2 = (2,0,0)
        p3 = (0,1,1)
        p4 = (0,2,2)
        self.assertEqual((sqrt(2), (0,0.5,0.5)), segment_segment_dist(p1,p2,p3,p4))

    def test_line_point(self):
        p = (0,0,1)
        l = ((1,1,0),(-1,-1,0))
        self.assertAlmostEqual(1., point_line_dist(p,l))

    def test_line_point2(self):
        p = (-3,9,7)
        l = ((0,9,2),(5,9,8))
        self.assertAlmostEqual(5.5056, point_line_dist(p,l), delta=0.001)

if __name__ == "__main__":
    unittest.main()
