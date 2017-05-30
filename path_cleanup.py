#!/usr/bin/env python3

import unittest
from math import sqrt

from rdp import rdp


def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

'''
    returns the closest distance between two line segments in 3D space.
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
        # t1 remains 0
        t2 = d/b if b>c else e/c
    else:
        t1 = (b*e-c*d)/D
        t2 = (a*e - b*d)/D

    # Is the closest point on the line bounded by the 2 sides of the segment?
    t1 = min(max(0,t1),1)
    t2 = min(max(0,t2),1)

    # difference between two closest points
    dP = (w[0]+t1*u[0]-t2*v[0], w[1]+t1*u[1]-t2*v[1], w[2]+t1*u[2]-t2*v[2])

    return sqrt(dot_product(dP, dP))


def cleanup(path, rdp_eps):
    # TODO pruning step

    # simplification step. Uses Ramer-Douglas-Peucker algorithm
    path = rdp(path, epsilon = rdp_eps)

    return path

class TestLineCalculations(unittest.TestCase):
    def test_perpendicular(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (0,0,1)
        p4 = (0,1,1)
        self.assertEqual(1, segment_segment_dist(p1,p2,p3,p4))

    def test_parallel(self):
        p1 = (0,0,0)
        p2 = (1,1,0)
        p3 = (0,0,1)
        p4 = (1,1,1)
        self.assertEqual(1, segment_segment_dist(p1,p2,p3,p4))

    def test_intersecting(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (0,0,0)
        p4 = (0,1,0)
        self.assertEqual(0, segment_segment_dist(p1,p2,p3,p4))

    def test_identical(self):
        p1 = (0,0,0)
        p2 = (1,1,0)
        self.assertEqual(0, segment_segment_dist(p1,p2,p1,p2))
        self.assertEqual(0, segment_segment_dist(p1,p2,p2,p1))

    def test_parallel_but_spaced_out(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (3,0,0)
        p4 = (4,0,0)
        self.assertEqual(3, segment_segment_dist(p1,p2,p3,p4))

    def test_perpendicular_but_spaced_out(self):
        p1 = (-2,0,0)
        p2 = (2,0,0)
        p3 = (0,1,1)
        p4 = (0,2,2)
        self.assertEqual(sqrt(2), segment_segment_dist(p1,p2,p3,p4))

if __name__ == "__main__":
    unittest.main()
