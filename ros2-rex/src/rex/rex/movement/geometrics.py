import numpy as np

def Rx(roll):
    """Rotation matrix around x (roll)"""
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1, 0,  0, 0],
                     [0, c, -s, 0],
                     [0, s,  c, 0],
                     [0, 0,  0, 1]])

def Ry(pitch):
    """Rotation matrix around y (pitch)"""
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]])

def Rz(yaw):
    """Rotation matrix around z (yaw)"""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def Rxyz(roll, pitch, yaw):
    if roll == 0 and pitch == 0 and yaw == 0:
        return np.identity(4)
    return np.dot(np.dot(Rx(roll), Ry(pitch)), Rz(yaw))

def RTmatrix(orientation, position):
    """Compose translation and rotation"""
    roll, pitch, yaw = orientation
    x0, y0, z0 = position
    
    translation = np.array([[1, 0, 0, x0],
                            [0, 1, 0, y0],
                            [0, 0, 1, z0],
                            [0, 0, 0, 1]])
    
    rotation = Rxyz(roll, pitch, yaw)
    return np.dot(rotation, translation)

def transform(coord, rotation, translation):
    """Transforms a vector to a desired rotation and translation"""
    vector = np.array([[coord[0]],
                       [coord[1]],
                       [coord[2]],
                       [1]])
    
    transform_vector = np.dot(RTmatrix(rotation, translation), vector)
    return transform_vector[:3, 0]
