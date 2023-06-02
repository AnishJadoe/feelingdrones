from feelingdrones.engineering.constants import *
import numpy as np

DEFAULT_LENGHTS = [LENGTH_LINK_1, LENGTH_LINK_2,LENGTH_LINK_3]

def apply_transform(base, transformations, transformation_index):

    transformations = reversed(transformations[:transformation_index+1])
    for transformation in transformations:
        base = transformation@base
    return base

# Define the forward kinematics function
def forward_kinematics(q):
    T01 = np.array([[np.cos(q[0,0]), -np.sin(q[0,0]), 0, 0],
                    [np.sin(q[0,0]), np.cos(q[0,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T12 = np.array([[np.cos(q[1,0]), -np.sin(q[1,0]), 0, LENGTH_LINK_1],
                    [np.sin(q[1,0]), np.cos(q[1,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T23 = np.array([[np.cos(q[2,0]), -np.sin(q[2,0]), 0, LENGTH_LINK_2],
                    [np.sin(q[2,0]), np.cos(q[2,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T34 = np.array([[1, 0, 0, LENGTH_LINK_3],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)
    return T04

def get_transformations(q, links_lengths=DEFAULT_LENGHTS):
    T01 = np.array([[np.cos(q[0,0]), -np.sin(q[0,0]), 0, 0],
                    [np.sin(q[0,0]), np.cos(q[0,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    T12 = np.array([[np.cos(q[1,0]), -np.sin(q[1,0]), 0, links_lengths[0] * 100],
                    [np.sin(q[1,0]), np.cos(q[1,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    T23 = np.array([[np.cos(q[2,0]), -np.sin(q[2,0]), 0, links_lengths[1] * 100],
                    [np.sin(q[2,0]), np.cos(q[2,0]), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    T34 = np.array([[1, 0, 0, links_lengths[2] * 100],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return [T01,T12,T23,T34]

def get_rotations(transformations, homogeneous=False):
    R01 = transformations[0][:3,:3]
    R12 = transformations[1][:3,:3]
    R23 = transformations[2][:3,:3]
    R34 = transformations[3][:3,:3]

    rotations = [R01,R12,R23,R34]
    if homogeneous:
        # preprocess rotations to be homogeneous 
        for i,rotation in enumerate(rotations):
            new_transform = np.eye(4)
            new_transform[:3,:3] = rotation
            rotations[i] = new_transform  

    return rotations