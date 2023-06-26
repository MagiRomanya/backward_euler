#!/usr/bin/env python3

from symulathon import Simulation
import numpy as np


def generate_parameters(k_border: tuple, k_bend_border: tuple) -> (list, list, list):
    """Generate cloth paramters interpolating between the ones defined at the corners.

    The function returns 3 lists:
    1st: normal spring paramters.
    2nd: bend spring parameters.
    3d: Indices relative to the full parameter vector [k, k_bend], telling
    where the k_border and k_bend_border are located.
    """
    sim = Simulation(1, 1)
    N, M = sim.getGridDimensions()
    mesh = sim.getMesh()
    pos = mesh.getPositions()
    vpos = np.array([pos[i:i+3] for i in range(0, len(pos), 3)])
    x, y, z = vpos.T
    # y plane
    maxX = np.max(x)
    maxZ = np.max(z)
    minX = np.min(x)
    minZ = np.min(z)
    deltaX = maxX - minX
    deltaZ = maxZ - minZ

    bottomleft, bottomright, topleft, topright = __find_cornersDIM(N, M)
    # print(bottomleft, bottomright, topleft, topright)
    # print(maxX, maxZ)
    # print(minX, minZ)

    spring_indices = sim.getSpringIndices()
    bspring_indices = sim.getBendSpringIndices()
    kspring = []
    kbendspring = []
    for i in range(0, len(spring_indices), 2):
        i1 = spring_indices[i]
        i2 = spring_indices[i+1]
        u1 = x[i1] / deltaX
        v1 = z[i1] / deltaZ
        u2 = x[i2] / deltaX
        v2 = z[i2] / deltaZ
        u = (u1 + u2) / 2
        v = (v1 + v2) / 2
        kspring.append(__interpolate(u, v, k_border))

    for i in range(0, len(bspring_indices), 2):
        i1 = bspring_indices[i]
        i2 = bspring_indices[i+1]
        u1 = x[i1] / deltaX
        v1 = z[i1] / deltaZ
        u2 = x[i2] / deltaX
        v2 = z[i2] / deltaZ
        u = (u1 + u2) / 2
        v = (v1 + v2) / 2
        kspring.append(__interpolate(u, v, k_bend_border))

    # Generate parameter list
    param_indices = [0] * 8
    bottomleft, bottomright, topleft, topright = __getCornerSpringNodes(N, M)
    param_indices[0] = __getSpringIndex(spring_indices, *bottomleft)
    param_indices[1] = __getSpringIndex(spring_indices, *bottomright)
    param_indices[2] = __getSpringIndex(spring_indices, *topleft)
    param_indices[3] = __getSpringIndex(spring_indices, *topright)
    bottomleft, bottomright, topleft, topright = __getCornerBendSpringNodes(N, M)
    param_indices[4] = __getSpringIndex(bspring_indices, *bottomleft)
    param_indices[5] = __getSpringIndex(bspring_indices, *bottomright)
    param_indices[6] = __getSpringIndex(bspring_indices, *topleft)
    param_indices[7] = __getSpringIndex(bspring_indices, *topright)

    return kspring, kbendspring, param_indices


def __interpolate(u: float, v: float, boundary):
    bottomleft, bottomright, topleft, topright = boundary
    w11 = (1-u) * (1-v)
    w12 = (1-u) * v
    w21 = u * (1 - v)
    w22 = u*v
    return bottomleft * w11 + bottomright * w12 +\
        topleft * w21 + topright * w22


def __find_pair(x, z, valuex, valuez):
    for i, xe in enumerate(x):
        if (valuex == xe):
            if (z[i] == valuez):
                return i
    print("FindPair: there are no coincidences")
    return False


def __find_corners(x: list, z: list):
    maxX = np.max(x)
    maxZ = np.max(z)
    minX = np.min(x)
    minZ = np.min(z)
    bottomleft = __find_pair(x, z, minX, minZ)
    bottomright = __find_pair(x, z, maxX, minZ)
    topleft = __find_pair(x, z, minX, maxZ)
    topright = __find_pair(x, z, maxX, maxZ)
    return bottomleft, bottomright, topleft, topright


def __find_cornersDIM(N: int, M: int):
    bottomleft, bottomright, topleft, topright = (0, M*(N-1), M-1, (M-1)*N + N-1)
    return bottomleft, bottomright, topleft, topright


def __getCornerSpringNodes(N: int, M: int):
    bl, br, tl, tr = __find_cornersDIM(N, M)
    bottomleft = (bl, bl+M)
    bottomright = (br, br-M)
    topleft = (tl, tl+M)
    topright = (tr, tr-M)
    return bottomleft, bottomright, topleft, topright


def __getCornerBendSpringNodes(N: int, M: int):
    bl, br, tl, tr = __find_cornersDIM(N, M)
    bottomleft = (bl, bl+M+1)
    bottomright = (br-M, br+1)
    topleft = (tl-1, tl+M)
    topright = (tr-M-1, tr)
    return bottomleft, bottomright, topleft, topright


def __getSpringIndex(spring_indices: list, a: int, b: int):
    for i in range(0, len(spring_indices), 2):
        i1 = spring_indices[i]
        i2 = spring_indices[i+1]
        if (i1 == a and i2 == b) or (i2 == a and i1 == b):
            return i
    print(f"UNEXPECTED: The index pair {a}, {b} does not form a spring in the mesh.")
    return False
