#!/usr/bin/env python3

from rdp import rdp


def cleanup(path, rdp_eps):
    # TODO pruning step
    
    # simplification step. Uses Ramer-Douglas-Peucker algorithm
    path = rdp(path, epsilon = rdp_eps)

    return path
