#!/usr/bin/python3
import sys
import distmesh as dm
import numpy as np
from signed_distance_factory import Grid, sphere, cylinder, box, union, intersection, difference,\
                                    translate, scale, rotate, erosion, dilation, opening
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from skimage import measure
from skimage.draw import ellipsoid
import mcubes
PATH_TO_LIBIGL = '/home/max/diku/2018/project2018b2/SoftRoboticDesign/robot_designs/libigl/python'
sys.path.insert(0, PATH_TO_LIBIGL)
import pyigl as igl

# Generate a level set about zero of two identical ellipsoids in 3D
ellip_base = ellipsoid(6, 10, 16, levelset=True)

verts, faces, normals, values = measure.marching_cubes_lewiner(ellip_base, 0)

def test_text(result):
    return "Success" if result else "Failed"

def test_same(f1, f2, grid_size):
    for i in grid_size[0]:
        for j in grid_size[1]:
            for k in grid_size[2]:
                if f1([i,j,k]) != f2([i,j,k]):
                    return False
    return True

# Test grid parameters
s = 0.5
min_coord = [-10., -10., -10.]
max_coord = [10., 10., 10.]
gran = int(1./s* (max(max_coord) - min(min_coord)) + 1)
grid_ = [np.arange(-10., 10., 20. / (float(gran) - 1)),
         np.arange(-10., 10., 20. / (float(gran) - 1)),
         np.arange(-10., 10., 20. / (float(gran) - 1))]
print(np.array(grid_).shape)

# Test 0: Creating a box
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(box(1,1,1))
b = box(1,1,1)
print("Test 0 (Create a box):", test_text(test_same(lambda x: G[x], b, grid_)))

# Test 1: Creating a cylinder
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(cylinder(1,1))
c = cylinder(1,1)
print("Test 1 (Create a cylinder):", test_text(test_same(lambda x: G[x], c, grid_)))

# Test 2: Creating a sphere
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(sphere(1))
s = sphere(1)
print("Test 2 (Create a sphere):", test_text(test_same(lambda x: G[x], s, grid_)))

# Test 3: Union
G1 = Grid(min_coord, max_coord, gran, gran, gran)
G1.create(box(2,2,2))
G2 = Grid(min_coord, max_coord, gran, gran, gran)
G2.create(box(1,1,1))

G = union(G1, G2)
G.create(G.phi)
b = box(2,2,2)
print("Test 3 (Union):", test_text(test_same(lambda x: G[x], b, grid_)))

# Test 4: Intersection
G1 = Grid(min_coord, max_coord, gran, gran, gran)
G1.create(box(2,2,2))
G2 = Grid(min_coord, max_coord, gran, gran, gran)
G2.create(box(1,1,1))

G = intersection(G1, G2)
G.create(G.phi)
b = box(1,1,1)
print("Test 4 (Intersection):", test_text(test_same(lambda x: G[x], b, grid_)))

# Test 5+6: Difference
G1 = Grid(min_coord, max_coord, gran, gran, gran)
G1.create(box(2,2,2))
G2 = Grid(min_coord, max_coord, gran, gran, gran)
G2.create(box(1,1,1))

G = difference(G1, G2)
G.create(G.phi)
b = box(2,2,2)
data_points = [[3.,3.,3.], [4.,4.,4.], [2,2,2], [1.5,1.5,1.5]]
print("Test 5 (Difference, same outside):",
       test_text(all(G[x] == b(x) for x in data_points)))
data_points = [[0,0,0], [0,0.5,0]]
print("Test 6 (Difference, different 'inside'):",
       test_text(all(not G[x] == b(x) for x in data_points)))

# Test 7: Translate
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(box(1,1,1))
before = all([G[0,0,0] == [-1.0], G[1,1,1] == [0.0]])
G = translate(G, [1,1,1])
G.create(G.phi)
print("Test 7 (Translation):", test_text(all([G[0,0,0] == [0.0], G[1,1,1] == [-1.0]]) and before))

# Test 8: Scale
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(box(1,1,1))
before = all([G[0,0,0] == [-1.0], G[1,1,1] == [0.0]])
G = scale(G, 2.0)
G.create(G.phi)
print("Test 8 (Scaling):", test_text(all([G[0,0,0] == [-2.0], G[1,1,1] == [-1.0]]) and before))

# Test 9: Rotate
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(cylinder(4,1))
c = cylinder(4,1)
before = all([G[0,0,0] == [-1.0], G[0,0,1] == [-1.0], G[0,0,2] == [0.0]])
# Rotation of 90 degrees about x-axis
R_x = np.array([[1,0,0],
                [0,0,-1],
                [0,1,0]])
G = rotate(G, R_x)
G.create(G.phi)
print("Test 9 (Rotate):",
      test_text(all([G[0,0,0] == [-1.0], G[0,0,1] == [0.0], G[0,0,2] == [1.0]]) and before))

# Test 10: Erosion (an erosion of a sphere is just a scaling)
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(sphere(4))
before = all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]])
G = erosion(G, 2.0)
G.create(G.phi)
print("Test 10 (Erosion):", test_text(all([G[0,0,0] == [-6.0], G[0,0,4] == [-2.0]]) and before))

# Test 11: Dilation
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(sphere(4))
before = all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]])
G = dilation(G, 2.0)
G.create(G.phi)
print("Test 11 (Dilation a):", test_text(all([G[0,0,0] == [-2.0], G[0,0,4] == [2.0]]) and before))

# Test 12: Opening
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(sphere(4))
before = all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]])
G = opening(G, 2.0)
G.create(G.phi)
print("Test 12 (Opening):", test_text(all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]]) and before))

# Test 13: Dilation
G = Grid(min_coord, max_coord, gran, gran, gran)
G.create(sphere(4))
before = all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]])
G = dilation(G, 5.0)
G.create(G.phi)
print([G[0,0,0] , G[0,0,4]])
print(np.min(G.values))
print("Test 13 (Dilation b):", test_text(all([G[0,0,0] == [-4.0], G[0,0,4] == [0.0]]) and before))

p, t = dm.distmeshnd(G.phi, dm.huniform, 0.2, (-1,-1,-1, 1,1,1))
