import math
import sys
import numpy as np
from stl import mesh
from functools import reduce

# Add pyigl to the path
PATH_TO_LIBIGL = '/home/max/diku/2018/project2018b2/SoftRoboticDesign/robot_designs/libigl/python'
sys.path.insert(0, PATH_TO_LIBIGL)
import pyigl as igl

'''
Implementation of a 3D-pyramid.

@param height: Height of pyramid
@param width: Width of pyramid
@param length: Length of pyramid
@param center: a point (x_0, y_0, z_0) that the cuboid is centered about
@param scale: Scale of the solid
@return: vertices and faces of the cuboid (V, F)
'''
def make_pyramid(height, width, length, center=(0.,0.,0.), scale=1.0):
    # For easier xml parsing
    if center is None:
        center = (0., 0., 0.)
    if scale is None:
        scale = 1.0

    if len(center) != 3:
        raise Exception("Center must be a 3-dimensional vector")

    V = np.array([[-width,0,-length],[width,0,-length],
                  [width,0,length],[-width,0,length],
                  [0.,height,0.]])
    V = (V * scale / 2.0 + center)
    # Faces of the pyramid
    F = np.array([[0,4,1],
                  [2,1,4],
                  [2,4,3],
                  [0,3,4],
                  [3,1,2],[3,0,1]], dtype=np.int32)
    return V, F


'''
Implementation of a 3D-cube.

@param size: 3-tuple with (x,y,z) lengths
@param center: a point (x_0, y_0, z_0) that the cuboid is centered about
@return: vertices and faces of the cuboid (V, F)
@param scale: Scale of the solid
'''
def make_cuboid(size, center=(0.,0.,0.), scale=1.0, refining=1):
    # For easier xml parsing
    if center is None:
        center = (0., 0., 0.)
    if scale is None:
        scale = 1.0

    if len(size) != 3:
        raise Exception("Size must have 3 components")

    if len(center) != 3:
        raise Exception("Center must be a 3-dimensional vector")

    x, y, z = size
    # The vertices of the cuboid
    V = np.array([[-x, y, z], [-x, y, -z], [x, y, -z], [x, y, z],
                  [x, -y, z], [x, -y, -z], [-x ,-y, -z], [-x, -y, z]])
    # Making sure the scale is correct and that the cube is centered around 'center'
    V = (V * scale / 2.0 + center)
    # Faces of the cuboid (ensures outward normals)
    F = np.array([[3,1,0],[3,2,1],
                  [4,2,3],[4,5,2],
                  [4,0,7],[4,3,0],
                  [7,5,4],[7,6,5],
                  [0,6,7],[0,1,6],
                  [6,2,5],[6,1,2]], dtype=np.int32)


    for _ in range(refining):
        FRefined = []
        VRefined = []
        for i,f in enumerate(F):
            v0 = V[f[0]]
            v1 = V[f[1]]
            v2 = V[f[2]]
            v3 = [(v0[0] + v1[0]) / 2.0, (v0[1] + v1[1]) / 2.0, (v0[2] + v1[2]) / 2.0]
            v4 = [(v1[0] + v2[0]) / 2.0, (v1[1] + v2[1]) / 2.0, (v1[2] + v2[2]) / 2.0]
            v5 = [(v2[0] + v0[0]) / 2.0, (v2[1] + v0[1]) / 2.0, (v2[2] + v0[2]) / 2.0]
            # Add the new vertices to VRefined
            VRefined.append(v0)
            VRefined.append(v1)
            VRefined.append(v2)
            VRefined.append(v3)
            VRefined.append(v4)
            VRefined.append(v5)
            # Add the new faces to FRefined
            idx = i * 6
            FRefined.append([idx, idx+3, idx+5])
            FRefined.append([idx+1, idx+4, idx+3])
            FRefined.append([idx+2, idx+5, idx+4])
            FRefined.append([idx+3, idx+4, idx+5])
        V = VRefined
        F = np.array(FRefined, dtype=np.int32)
    return V, F

'''
Implementation for approximating a cylinder.
@param start: The center of the start of the cylinder
@param end: The center of the end of the cylinder
@param scale: The scale of the cylinder
@param resolution: The amount of points used to approximate circles
@return: vertices and faces of the cylinder (V, F)
'''
def make_cylinder(start, end, radius=1.0, resolution=8):
    # For easier xml parsing
    if radius is None:
        radius = 1.0
    if resolution is None:
        resolution = 8

    if len(start) != 3:
        raise Exception("Center must be a 3-dimensional vector")

    if len(end) != 3:
        raise Exception("Center must be a 3-dimensional vector")

    # Figure out which is start and which is end
    diff = [start[i] - end[i] for i in range(len(start))]
    if reduce(lambda x1, x2: x1+x2, diff) > 0:
        tmp = end
        end = start
        start = tmp

    xs, ys, zs = start # Center of the start of the clyinder
    xe, ye, ze = end   # Center of the end of the cylinder
    if xs > xe or ys > ye or zs > ze:
        xs, ys, zs = end   # Center of the end of the clyinder
        xe, ye, ze = start   # Center of the start of the cylinder
    # Depending on orientation, determine circle points
    if xs - xe != 0:
        V = [[xs, ys, zs]] # The center
        for i in range(resolution):
            rotation = -(2 * np.pi / resolution) * i
            ys_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + ys
            zs_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + zs
            V.append([xs, ys_rotated, zs_rotated])
        V.append([xe, ye, ze]) # The center
        for i in range(resolution):
            rotation = -(2 * np.pi / resolution) * i
            ye_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + ye
            ze_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + ze
            V.append([xe, ye_rotated, ze_rotated])

    if ys - ye != 0:
        V = [[xs, ys, zs]] # The center
        for i in range(resolution):
            rotation = (2 * np.pi / resolution) * i
            xs_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + xs
            zs_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + zs
            V.append([xs_rotated, ys, zs_rotated])
        # Vertices of the "end circle"
        V.append([xe, ye, ze]) # The center
        for i in range(resolution):
            rotation = (2 * np.pi / resolution) * i
            xe_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + xe
            ze_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + ze
            V.append([xe_rotated, ye, ze_rotated])

    if zs - ze != 0:
        V = [[xs, ys, zs]] # The center
        for i in range(resolution):
            rotation = -(2 * np.pi / resolution) * i
            xs_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + xs
            ys_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + ys
            V.append([xs_rotated, ys_rotated, zs])
        # Vertices of the "end circle"
        V.append([xe, ye, ze]) # The center
        for i in range(resolution):
            rotation = -(2 * np.pi / resolution) * i
            xe_rotated = (radius * np.cos(rotation) - 0 * np.sin(rotation)) + xe
            ye_rotated = (0 * np.cos(rotation) + radius * np.sin(rotation)) + ye
            V.append([xe_rotated, ye_rotated, ze])
    V = np.array(V)
    # Computing faces
    F = []
    # Computing faces for the start circle
    for i in range(1, resolution+1):
        # 0 := center
        if i == resolution:
            F.append([0, i, 1])
        else:
            F.append([0, i, i+1])
    # Computing faces for the end circle
    for i in range(1, resolution+1):
        # resolution := center
        if i == resolution:
            F.append([resolution+1, resolution+2, resolution+1+i])
        else:
            F.append([resolution+1, resolution+2+i, resolution+1+i])
    # Computing faces for the sides connecting the circles
    for i in range(1, resolution+1):
        if i == resolution:
            F.append([resolution, resolution*2+1, resolution+2])
            F.append([resolution+2, 1, resolution])
        else:
            F.append([i, resolution+1+i, resolution+2+i])
            F.append([resolution+2+i, i+1, i])
    F = np.array(F, np.int32)
    return V, F

'''
Implementation of a Icosahedron for approximating a sphere.

@param center: The center of the sphere
@param scale: The scale of the sphere
@param refining: The amount of times the triangles of the Icosahedron should be subdivided.
@return: vertices and faces of the sphere (V, F)
'''
def make_sphere(center, scale=1.0, refining=1):
    if len(center) != 3:
        raise Exception("Center must be a 3-dimensional vector")
    # Initial 12 vertices of the sphere
    x, y, z = center
    # Vertices
    V = []
    # The golden ratio
    t = (1.0 + np.sqrt(5.0)) / 2.0

    # First rectangle
    V.append([-scale, t*scale, 0.])
    V.append([scale, t*scale, 0.])
    V.append([-scale, -t*scale, 0.])
    V.append([scale, -t*scale, 0.])

    # Second rectangle
    V.append([0., -scale, t*scale])
    V.append([0., scale, t*scale])
    V.append([0., -scale, -t*scale])
    V.append([0., scale, -t*scale])

    # Third rectangle
    V.append([t*scale, 0., -scale])
    V.append([t*scale, 0., scale])
    V.append([-t*scale, 0., -scale])
    V.append([-t*scale, 0., scale])

    # Building initial triangles
    F = []
    # 5 aces around point 0
    F.append([0, 11, 5])
    F.append([0, 5, 1])
    F.append([0, 1, 7])
    F.append([0, 7, 10])
    F.append([0, 10, 11])
    # 5 adjacent faces
    F.append([1, 5, 9])
    F.append([5, 11, 4])
    F.append([11, 10, 2])
    F.append([10, 7, 6])
    F.append([7, 1, 8])
    # 5 aces around point 3
    F.append([3, 9, 4])
    F.append([3, 4, 2])
    F.append([3, 2, 6])
    F.append([3, 6, 8])
    F.append([3, 8, 9])
    # 5 adjacent faces
    F.append([4, 9, 5])
    F.append([2, 4, 11])
    F.append([6, 2, 10])
    F.append([8, 6, 7])
    F.append([9, 8, 1])

    # Refining the Icosahedron by subdividing the triangles
    for _ in range(refining):
        FRefined = []
        VRefined = []
        for i,f in enumerate(F):
            v0 = V[f[0]]
            v1 = V[f[1]]
            v2 = V[f[2]]
            # V3
            v3 = [(v0[0] + v1[0]) / 2.0, (v0[1] + v1[1]) / 2.0, (v0[2] + v1[2]) / 2.0]
            v3_len = np.sqrt(v3[0]**2 + v3[1]**2 + v3[2]**2)
            # Rescaling of V3
            v3 = [v3[0]*np.sqrt(t*t+1)*scale / v3_len,
                  v3[1]*np.sqrt(t*t+1)*scale / v3_len,
                  v3[2]*np.sqrt(t*t+1)*scale / v3_len]
            # V4
            v4 = [(v1[0] + v2[0]) / 2.0, (v1[1] + v2[1]) / 2.0, (v1[2] + v2[2]) / 2.0]
            v4_len = np.sqrt(v4[0]**2 + v4[1]**2 + v4[2]**2)
            # Rescaling of V4
            v4 = [v4[0]*np.sqrt(t*t+1)*scale / v4_len,
                  v4[1]*np.sqrt(t*t+1)*scale / v4_len,
                  v4[2]*np.sqrt(t*t+1)*scale / v4_len]
            # V5
            v5 = [(v2[0] + v0[0]) / 2.0, (v2[1] + v0[1]) / 2.0, (v2[2] + v0[2]) / 2.0]
            v5_len = np.sqrt(v5[0]**2 + v5[1]**2 + v5[2]**2)
            # Rescaling of V5
            v5 = [v5[0]*np.sqrt(t*t+1)*scale / v5_len,
                  v5[1]*np.sqrt(t*t+1)*scale / v5_len,
                  v5[2]*np.sqrt(t*t+1)*scale / v5_len]
            #v4 = 0.5 * (v1 + v2) / np.linalg.norm(v1 + v2)
            #v5 = 0.5 * (v2 + v0) / np.linalg.norm(v2 + v0)
            # Add the new vertices to VRefined
            VRefined.append(v0)
            VRefined.append(v1)
            VRefined.append(v2)
            VRefined.append(v3)
            VRefined.append(v4)
            VRefined.append(v5)
            # Add the new faces to FRefined
            idx = i * 6
            FRefined.append([idx, idx+3, idx+5])
            FRefined.append([idx+1, idx+4, idx+3])
            FRefined.append([idx+2, idx+5, idx+4])
            FRefined.append([idx+3, idx+4, idx+5])
        V = VRefined
        F = FRefined

    # Centering it around "center"
    V = np.array(V)
    F = np.array(F, dtype=np.int32)

    return V, F

'''
Author: Jens Kanstrup Larsen <jensklmail@yahoo.dk>

@param center: a point (x,y,z) that the prism is centered on (see above)
@param l: the length (y-axis) of the prism
@param h: the height (z-axis) of the prism
@param v: the angle of the bottom prism wedge (in degrees)
@return: vertices and faces of the prism (V, F)
'''
def make_prism(center=(0.,0.,0.), l=1.0, h=1.0, v=45.0):
    if len(center) != 3:
        raise Exception("Center must be a 3-dimensional vector")
    if v >= 180:
        raise Exception("Angle must be less than 180 degrees")

    x,y,z = center
    w = 2*((h/math.cos(math.radians(v/2)))*math.sin(math.radians(v/2)))

    # The vertices of the prism
    V = np.array([[x-w/2, y+l/2, z+h/2], [x, y+l/2, z-h/2], [x+w/2, y+l/2, z+h/2],
                  [x+w/2, y-l/2, z+h/2], [x, y-l/2, z-h/2], [x-w/2, y-l/2, z+h/2]])

    # Faces of the prism (ensures outward normals)
    F = np.array([[0,2,1],[3,5,4],
                  [0,3,2],[0,5,3],
                  [1,3,4],[1,2,3],
                  [0,1,4],[0,4,5]], dtype=np.int32)

    # Refining the faces by subdividing triangles
    for _ in range(1):
        FRefined = []
        VRefined = []
        for i,f in enumerate(F):
            v0 = V[f[0]]
            v1 = V[f[1]]
            v2 = V[f[2]]
            # V3
            v3 = [(v0[0] + v1[0]) / 2.0, (v0[1] + v1[1]) / 2.0, (v0[2] + v1[2]) / 2.0]
            v4 = [(v1[0] + v2[0]) / 2.0, (v1[1] + v2[1]) / 2.0, (v1[2] + v2[2]) / 2.0]
            # V5
            v5 = [(v2[0] + v0[0]) / 2.0, (v2[1] + v0[1]) / 2.0, (v2[2] + v0[2]) / 2.0]
            # Add the new vertices to VRefined
            VRefined.append(v0)
            VRefined.append(v1)
            VRefined.append(v2)
            VRefined.append(v3)
            VRefined.append(v4)
            VRefined.append(v5)
            # Add the new faces to FRefined
            idx = i * 6
            FRefined.append([idx, idx+3, idx+5])
            FRefined.append([idx+1, idx+4, idx+3])
            FRefined.append([idx+2, idx+5, idx+4])
            FRefined.append([idx+3, idx+4, idx+5])
        V = VRefined
        F = np.array(FRefined, dtype=np.int32)

    return V, F
