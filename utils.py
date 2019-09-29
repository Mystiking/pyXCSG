import sys
import numpy as np
from stl import mesh

# Add pyigl to the path
PATH_TO_LIBIGL = '/home/max/diku/2018/project2018b2/SoftRoboticDesign/robot_designs/libigl/python'
sys.path.insert(0, PATH_TO_LIBIGL)
import pyigl as igl

'''
Save a model given as vertices and faces as an .stl file.

@param fname: Filename to save it to
@param V: Vertices of the model
@param F: Faces of the model
'''
def save_to_stl(fname, V, F):
    solid = mesh.Mesh(np.zeros(F.shape[0], dtype=mesh.Mesh.dtype))
    for i,f in enumerate(F):
        for j in range(3):
            solid.vectors[i][j] = V[f[j], :]
    solid.save(fname)

'''
Translate vertices V by dist.

@param V: Vertices (as igl.eigen.MatrixXd)
@param dist: distances [distX, distY, distZ]
@return: Translated vertices (as igl.eigen.MatrixXd)
'''
def translate(V, dist):
    return igl.eigen.MatrixXd(np.array(V) + dist)

'''
Rotates vertices V around its own center in the X-axis direction.

@param V: Vertices (as igl.eigen.MatrixXd)
@param rotation: radians to rotate
@return: Rotated vertices (as igl.eigen.MatrixXd)
'''
def rotateX(V, rotation):
    V_rotated = []
    centerx, centery, centerz = np.mean(np.array(V), axis=0)
    for v in np.array(V):
        x, y, z = v
        yr = ((y - centery) * np.cos(rotation) - (z - centerz) * np.sin(rotation)) + centery
        zr = ((y - centery) * np.sin(rotation) + (z - centerz) * np.cos(rotation)) + centerz
        V_rotated.append([x, yr, zr])
    return igl.eigen.MatrixXd(np.array(V_rotated))

'''
Rotates vertices V around its own center in the X-axis direction.

@param V: Vertices (as igl.eigen.MatrixXd)
@param rotation: radians to rotate
@return: Rotated vertices (as igl.eigen.MatrixXd)
'''
def rotateY(V, rotation):
    V_rotated = []
    centerx, centery, centerz = np.mean(np.array(V), axis=0)
    for v in np.array(V):
        x, y, z = v
        zr = ((z - centerz) * np.cos(rotation) - (x - centerx) * np.sin(rotation)) + centerz
        xr = ((z - centerz) * np.sin(rotation) + (x - centerx) * np.cos(rotation)) + centerx
        V_rotated.append([xr, y, zr])
    return igl.eigen.MatrixXd(np.array(V_rotated))

'''
Rotates vertices V around its own center in the X-axis direction.

@param V: Vertices (as igl.eigen.MatrixXd)
@param rotation: radians to rotate
@return: Rotated vertices (as igl.eigen.MatrixXd)
'''
def rotateZ(V, rotation):
    V_rotated = []
    centerx, centery, centerz = np.mean(np.array(V), axis=0)
    for v in np.array(V):
        x, y, z = v
        xr = ((x - centerx) * np.cos(rotation) - (y - centery) * np.sin(rotation)) + centerx
        yr = ((x - centerx) * np.sin(rotation) + (y - centery) * np.cos(rotation)) + centery
        V_rotated.append([xr, yr, z])
    return igl.eigen.MatrixXd(np.array(V_rotated))

'''
Scale vertices V by 'scale'

@param V: Vertices (as igl.eigen.MatrixXd)
@param scale: scaling factor
@return: Scaled vertices (as igl.eigen.MatrixXd)
'''
def scale(V, scale):
    return igl.eigen.MatrixXd(np.array(V) * scale)

'''
Scale vertices of a rectangels V by 'scale' in the xz-direction

@param V: Vertices (as igl.eigen.MatrixXd)
@param scale: scaling factor
@return: Scaled vertices (as igl.eigen.MatrixXd)
'''
def scaleRectangleXZ(V, scale):
    V_scaled = []
    xy = [1, 3, 4, 6]
    for i, v in enumerate(np.array(V)):
        if i in xy:
            V_scaled.append(v * scale)
        else:
            V_scaled.append(v)
    return igl.eigen.MatrixXd(np.array(V_scaled))
