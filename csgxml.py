#!/usr/bin/python3
import xml.etree.ElementTree as ET
import sys, os
import numpy as np
from factory import make_cuboid, make_cylinder, make_sphere, make_pyramid, make_prism
from utils import save_to_stl, translate, rotateX, rotateY, rotateZ, scale, scaleRectangleXZ

# Add pyigl to the path
PATH_TO_LIBIGL = '/home/max/diku/2018/project2018b2/SoftRoboticDesign/robot_designs/libigl/python'
sys.path.insert(0, PATH_TO_LIBIGL)
import pyigl as igl


def parse_cuboid(element):
    # Size parameter (required)
    size = element.find('size').text
    size = list(map(float, size[size.find('[')+1:size.find(']')].split(',')))
    # Center parameter (optional)
    center = element.find('center')
    if not center is None:
        center = list(map(float, center.text[center.text.find('[')+1:center.text.find(']')].split(',')))

    # Scale parameter (optional)
    scale = element.find('scale')
    if not scale is None:
        scale = float(scale.text)
    # Refining parameter (optional)
    refining = element.find('refining')
    if not refining is None:
        refining = int(refining.text)
    else:
        refining = 1
    V, F = make_cuboid(size, center, scale, refining=refining)
    return [igl.eigen.MatrixXd(V), igl.eigen.MatrixXi(F)]

def parse_cylinder(element):
    # Start parameter (required)
    start = element.find('start').text
    start = list(map(float, start[start.find('[')+1:start.find(']')].split(',')))
    # End parameter (required)
    end = element.find('end').text
    end = list(map(float, end[end.find('[')+1:end.find(']')].split(',')))
    # Radius parameter (optional)
    radius = element.find('radius')
    if not radius is None:
        radius = float(radius.text)
    # Resolution parameter (optional)
    resolution = element.find('resolution')
    if not resolution is None:
        resolution = int(resolution.text)
    V, F = make_cylinder(start, end, radius, resolution)
    return [igl.eigen.MatrixXd(V), igl.eigen.MatrixXi(F)]

def parse_sphere(element):
    # Center parameter (required)
    center = element.find('center').text
    center = list(map(float, center[center.find('[')+1:center.find(']')].split(',')))
    # Scale parameter (optional)
    scale = element.find('scale')
    if not scale is None:
        scale = float(scale.text)
    # Refine parameter (optional)
    refine = element.find('refining')
    if not refine is None:
        refine = int(refine.text)
    V, F = make_sphere(center, scale, refine)
    return [igl.eigen.MatrixXd(V), igl.eigen.MatrixXi(F)]

def parse_pyramid(element):
    height = float(element.find('height').text)
    width = float(element.find('width').text)
    length = float(element.find('length').text)
    # Center parameter (required)
    center = element.find('center')
    if not center is None:
        center = center.text
        center = list(map(float, center[center.find('[')+1:center.find(']')].split(',')))
    # Scale parameter (optional)
    scale = element.find('scale')
    if not scale is None:
        scale = float(scale.text)
    V, F = make_pyramid(height, width, length, center, scale)
    return [igl.eigen.MatrixXd(V), igl.eigen.MatrixXi(F)]

def parse_prism(element):
    center = element.find('center')
    if not center is None:
        center = list(map(float, center.text[center.text.find('[')+1:center.text.find(']')].split(',')))

    length = element.find('length')
    if not length is None:
        length = float(length.text)

    height = element.find('height')
    if not height is None:
        height = float(height.text)

    angle = element.find('angle')
    if not angle is None:
        angle = float(angle.text)

    V, F = make_prism(center, length, height, angle)
    return [igl.eigen.MatrixXd(V), igl.eigen.MatrixXi(F)]


def parse_translation(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Distance parameter (required)
    distance = element.find('distance').text
    distance = list(map(float, distance[distance.find('[')+1:distance.find(']')].split(',')))
    # Translate and return the "new" vertices
    return translate(V, distance), F

def parse_rotationX(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Rotation parameter (required)
    rotation = float(element.find('rotation').text)
    # Transform the degrees into radians
    rotation = rotation * np.pi / 180.
    # Translate and return the "new" vertices
    return rotateX(V, rotation), F

def parse_rotationY(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Rotation parameter (required)
    rotation = float(element.find('rotation').text)
    # Transform the degrees into radians
    rotation = rotation * np.pi / 180.
    # Translate and return the "new" vertices
    return rotateY(V, rotation), F

def parse_rotationZ(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Rotation parameter (required)
    rotation = float(element.find('rotation').text)
    # Transform the degrees into radians
    rotation = rotation * np.pi / 180.
    # Translate and return the "new" vertices
    return rotateZ(V, rotation), F

def parse_scale(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Scale parameter (required)
    s = element.find('scale').text
    s = list(map(float, s[s.find('[')+1:s.find(']')].split(',')))
    # Translate and return the "new" vertices
    return scale(V, s), F


def parse_scale_rectangleXZ(element, csg_graph):
    # Get the current csg object
    V, F = csg_graph[element.find('operand').text.strip()]
    # Scale parameter (required)
    s = element.find('scale').text
    s = list(map(float, s[s.find('[')+1:s.find(']')].split(',')))
    # Translate and return the "new" vertices
    return scaleRectangleXZ(V, s), F


def parse_union(element, csg_graph):
    # Placeholders for the new vertices and faces
    V = igl.eigen.MatrixXd()
    F = igl.eigen.MatrixXi()
    # Get the operand names
    operands = element.findall('operand')
    operand_1 = operands[0].text.strip()
    operand_2 = operands[1].text.strip()
    # Perform the Union
    V1, F1 = csg_graph[operand_1]
    V2, F2 = csg_graph[operand_2]

    igl.cgal.mesh_boolean(V1, F1, V2, F2, igl.MESH_BOOLEAN_TYPE_UNION, V, F)

    return V,F

def parse_intersection(element, csg_graph):
    # Placeholders for the new vertices and faces
    V = igl.eigen.MatrixXd()
    F = igl.eigen.MatrixXi()
    # Get the operand names
    operands = element.findall('operand')
    operand_1 = operands[0].text.strip()
    operand_2 = operands[1].text.strip()
    # Perform the Union
    V1, F1 = csg_graph[operand_1]
    V2, F2 = csg_graph[operand_2]

    igl.cgal.mesh_boolean(V1, F1, V2, F2, igl.MESH_BOOLEAN_TYPE_INTERSECT, V, F)

    return V,F

def parse_difference(element, csg_graph):
    # Placeholders for the new vertices and faces
    V = igl.eigen.MatrixXd()
    F = igl.eigen.MatrixXi()
    # Get the operand names
    operands = element.findall('operand')
    operand_1 = operands[0].text.strip()
    operand_2 = operands[1].text.strip()
    # Perform the Union
    V1, F1 = csg_graph[operand_1]
    V2, F2 = csg_graph[operand_2]

    igl.cgal.mesh_boolean(V1, F1, V2, F2, igl.MESH_BOOLEAN_TYPE_MINUS, V, F)

    return V,F

parse_shape = {'cube' : lambda e: parse_cuboid(e),
               'cylinder': lambda e: parse_cylinder(e),
               'sphere': lambda e: parse_sphere(e),
               'prism': lambda e: parse_prism(e),
               'pyramid': lambda e: parse_pyramid(e)}

parse_unaries = {'translate' : lambda e,csg_graph: parse_translation(e, csg_graph),
                 'rotateX' : lambda e,csg_graph: parse_rotationX(e, csg_graph),
                 'rotateY' : lambda e,csg_graph: parse_rotationY(e, csg_graph),
                 'rotateZ' : lambda e,csg_graph: parse_rotationZ(e, csg_graph),
                 'scale': lambda e, csg_graph: parse_scale(e, csg_graph),
                 'scaleRectangleXZ': lambda e,csg_graph: parse_scale_rectangleXZ(e, csg_graph)}


parse_operations = {'union' : lambda e,csg_graph: parse_union(e, csg_graph),
                    'intersection' : lambda e,csg_graph: parse_intersection(e,csg_graph),
                    'difference' : lambda e,csg_graph: parse_difference(e,csg_graph)}

def parse_csg_graph(xml_file):
    # Placeholder which will contain all nodes of the csg graph
    csg_graph = {}
    # The tree from the xml file
    tree = ET.parse(xml_file)
    root = tree.getroot()
    last_name = ''
    for child in root:
        if child.tag == 'solid':
            # Get the identifier of the object
            name = child.attrib['name']
            # Shape name
            shape = child.attrib['shape']
            solid = parse_shape[shape](child)
            csg_graph[name] = solid
        elif child.tag == 'unary_op':
            # Get the identifier of the object
            name = child.attrib['name']
            # Operation to perform
            unary = child.attrib['type']
            solid = parse_unaries[unary](child, csg_graph)
            csg_graph[name] = solid
        elif child.tag == 'binary_op':
            # Get the identifier of the object
            name = child.attrib['name']
            # Operation to perform
            op = child.attrib['type']
            solid = parse_operations[op](child, csg_graph)
            csg_graph[name] = solid
        else:
            raise Exception("Unknown xml format")
        last_name = name
    return csg_graph, last_name

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: csgxml.py [filename]")
        exit()

    FNAME = sys.argv[1]
    csg_graph, last_name = parse_csg_graph(FNAME)
    VLast, FLast = csg_graph[last_name]
    print(last_name)

    save = False
    if len(sys.argv) > 2:
        save = True
        stl_name = sys.argv[2]

    viewer = igl.glfw.Viewer()
    viewer.data().clear()
    viewer.data().set_mesh(VLast, FLast)
    viewer.data().show_lines = True
    viewer.launch()

    if save:
        save_to_stl(stl_name, np.array(VLast), np.array(FLast))
