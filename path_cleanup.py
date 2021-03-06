#!/usr/bin/env python3

import copy
import itertools
import time
import unittest
from math import sin, sqrt

from bitarray import bitarray

### tuning variables ###

position_delta = 2. # how many meters to move before appending a new position to return_path
pruning_delta = position_delta * 1.5 # how many meters apart must two points be, such that we can assume there is no obstacle between those points
rdp_epsilon = position_delta * 0.5
max_path_len = 100

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
        return float("inf"), ([0,0],[0,0])
    else:
        t1 = (b*e-c*d)/D
        t2 = (a*e-b*d)/D

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

    area = sqrt(max(0,s*(s-a)*(s-b)*(s-c))) # inner part must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.

    return 2*area/b

'''
    This is a simplification algorithm, which generates a bitmask that says which points to keep (1) and which to delete (0).
    For details on how it works, see the wikipedia article on the Ramer-Douglas-Peucker algorithm.

    The epsilon value defines how aggressive the simplification is.

    The allowed_time defines how long the algorithm will run before returning, in ms.

    This method returns True if the algorithm has run to completion (and the bitmask is optimal), and False otherwise.
    If this method returns false, running it again will likely produce a bitmask with more False values. That said, it is
    an anytime algorithm, which never returns something incorrect. It is always possible to use the bitmask to simplify
    the path.
'''
stk, bitmask = None, None
def rdp_iter(path, epsilon, allowed_time):
    global stk, bitmask
    if stk is None and bitmask is None:
        # reset state to starting state
        stk = []
        end = len(path)-1
        bitmask = bitarray([True]*(end+1))
        stk.append( (0, end) )

    start_time = time.clock()
    while stk:
        if (time.clock()-start_time > allowed_time/1000.):
            return False
        start, end = stk.pop()
        max_dist = 0.
        max_index = start
        for i in range(max_index+1, end):
            if bitmask[i]:
                dist = point_line_dist(path[i], (path[start], path[end]))
                if dist > max_dist:
                    max_index = i
                    max_dist = dist
        if max_dist > epsilon:
            stk.append( (start, max_index) )
            stk.append( (max_index, end) )
        else:
            for i in range(start+1, end):
                bitmask[i] = False

    return True

'''
    Saves a list of tuples to memory, each representing a detected loop. The tuple looks like (a,b,c), where a is the index of the first item to remove,
    b-1 is the index of the last item to remove (b itself should stay), and c is the point (as a tuple) which represents the new point to be inserted in that place

    This method will never detect a loop that is wholly contained within another loop

    This method takes as input: 1. an index which tells the algorithm where to continue searching from, and 2. how much time it can search before it must return (in ms)

    This method returns the index of where the algorithm left off. If a -1 is returned, the algorithm has run to completion.
'''
detected_loops = []
def detect_loops(path, resume_state, allowed_time):
    global detected_loops
    start_time = time.clock()
    min_j = resume_state[1] # this is to prevent searching for loops that end before an existing loop, which prevents loops-within-loops.
    for i in range(resume_state[0] or 1, len(path)-1): # we will start at the specified index. If None or 0 is specified, it will start at 1.
        if time.clock()-start_time > allowed_time/1000.:
            return (i, min_j)
        # here we can choose: prune big/small loops starting in front/back?
        # for j in range(len(path)-2, i+1,-1): # counts backwards. This prunes old, big loops first.
        for j in range(max(min_j,i+2), len(path)-1): # count forwards. This prunes old, small loops first.
            dist = segment_segment_dist(path[i], path[i+1], path[j], path[j+1])
            if dist[0] <= pruning_delta:
                min_j = j
                # path = path[:i+1] + [dist[1]] + path[j+1:]
                detected_loops.append( (i+1, j+1, dist[1]) )
    return (-1, 0)

'''
    Takes in a list and return the same list with all instances of 'item' removed
'''
def remove_matching(arr, item):
    return [value for value in arr if value != item]

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
'''
class Path:
    def __init__(self, path):
        self.path = path
        self.worst_length = 0

    def append_if_far_enough(self, p):
        if len(self.path) > self.worst_length:
            self.worst_length = len(self.path)

        x,y,z = p
        x_old, y_old, z_old = self.path[-1]
        if (x-x_old)**2+(y-y_old)**2+(z-z_old)**2 >= position_delta**2:
            self.path.append(p)

    '''
        Call this method regularly to clean up the path in memory.
    '''
    def routine_cleanup(self):
        if len(self.path) < max_path_len - 10:
            return

        # detect loops
        global detected_loops
        detected_loops = []
        loop_algorithm_state = (0,0) # stores how far along the loop-finding algorithm has searched
        while loop_algorithm_state[0] != -1:
            # allow this algorithm 0.5ms to run, and then run it repeatedly until it reports that it has completed
            loop_algorithm_state = detect_loops(self.path, (loop_algorithm_state), 0.5)
        loops = copy.deepcopy(detected_loops)

        prune_size_dict = {}
        ignored_points = 0 # this is the number of points that are going to get added back
        for (a,b,c) in loops:
            for i in range(a,b):
                prune_size_dict[i] = None
            ignored_points += 1
        potential_amount_to_prune = len(prune_size_dict.keys()) - ignored_points

        # simplify
        global stk, bitmask
        stk = None
        bitmask = None
        while not rdp_iter(self.path, rdp_epsilon, 0.5):
            pass
        simplification_bitmask = bitmask
        potential_amount_to_simplify = len(simplification_bitmask) - simplification_bitmask.count()


        if potential_amount_to_simplify > 10: # if applying simplification would remove 10+ points
            # just run simplification
            self.path = list(itertools.compress(self.path, simplification_bitmask))
        elif potential_amount_to_prune:
            pruned_points = 0
            while pruned_points < 10:
                loop = loops.pop(0) # start pruning loops from the beginning
                for i in range(loop[0], loop[1]):
                    self.path[i] = None
                pruned_points += loop[1] - loop[0] - 1
        elif potential_amount_to_simplify + potential_amount_to_prune > 5:
            self.path = get_flyback_path()
        else: # can't clean up any more
            raise Exception("Out of Memory. Safe RTL unavailabe.")

        self.path = remove_matching(self.path, None)

    '''
        Hypothetically, if the copter were to fly back now, what path would it fly? This runs an aggressive cleanup and returns a path,
        but it does not alter the path in memory.
    '''
    def get_flyback_path(self):
        ret = copy.deepcopy(self.path)

        # detect loops
        global detected_loops
        detected_loops = []
        loop_algorithm_state = (0,0) # stores how far along the loop-finding algorithm has searched
        while loop_algorithm_state[0] != -1:
            # allow this algorithm 500ms to run, and then run it repeatedly until it reports that it has completed
            loop_algorithm_state = detect_loops(ret, loop_algorithm_state, 0.5)
        loops = copy.deepcopy(detected_loops)

        # simplify
        global stk, bitmask
        stk = None
        bitmask = None
        while not rdp_iter(self.path, rdp_epsilon, 0.5):
            pass
        simplification_bitmask = bitmask
        potential_amount_to_simplify = len(simplification_bitmask) - simplification_bitmask.count()

        # flag points for simplification removal
        for i in range(len(ret)):
            if not simplification_bitmask[i]:
                ret[i] = None
        # flag points for pruning removal
        for a,b,_ in loops:
            for i in range(a,b):
                ret[i] = None
        # finally, put all the new in-between numbers where they belong.
        for a,b,c in loops:
            ret[int((a+b)/2.)] = c

        return remove_matching(ret, None) # remove all null-valued points before returning

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
        self.assertEqual((float('inf'), ([0, 0], [0, 0])), segment_segment_dist(p1,p2,p3,p4))

    def test_intersecting(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (0,0,0)
        p4 = (0,1,0)
        self.assertEqual((0,(0,0,0)), segment_segment_dist(p1,p2,p3,p4))

    def test_identical(self):
        p1 = (0,0,0)
        p2 = (1,1,0)
        self.assertEqual((float('inf'), ([0, 0], [0, 0])), segment_segment_dist(p1,p2,p1,p2))
        self.assertEqual((float('inf'), ([0, 0], [0, 0])), segment_segment_dist(p1,p2,p2,p1))

    def test_parallel_but_spaced_out(self):
        p1 = (0,0,0)
        p2 = (1,0,0)
        p3 = (3,0,0)
        p4 = (4,0,0)
        self.assertEqual((float('inf'), ([0, 0], [0, 0])), segment_segment_dist(p1,p2,p3,p4))

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

    def test_recursive_rdp(self):
        inp = [(0,0,0), (1,4,6), (4,2,1), (4,2,2), (4,3,3), (5,3,3), (6,6,9)]
        out = [(0, 0, 0), (1, 4, 6), (4, 2, 1), (6, 6, 9)]
        self.assertEqual(out, rdp(inp, 1))

    def test_iterative_rdp(self):
        inp = [(0,0,0), (1,4,6), (4,2,1), (4,2,2), (4,3,3), (5,3,3), (6,6,9)]
        out = [(0, 0, 0), (1, 4, 6), (4, 2, 1), (6, 6, 9)]
        self.assertEqual(out, rdp_iter(inp, 1))

if __name__ == "__main__":
    unittest.main()
