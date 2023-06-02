import numpy as np
from feelingdrones.engineering.constants import *

def get_gravity_matrix(q):
    return  GRAIVTY_CONSTANT * np.array([[-(MASS_LINK_1+ MASS_LINK_2+MASS_LINK_3 )*LENGTH_LINK_1*np.sin(q[0,0]) - (MASS_LINK_2+MASS_LINK_3)*LENGTH_LINK_2*np.sin(q[0,0]+q[1,0]) 
                                  - MASS_LINK_3*LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0])], [- (MASS_LINK_2+MASS_LINK_3) * LENGTH_LINK_2 * np.sin(q[1,0]+q[2,0]) - MASS_LINK_3*LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0])],
                                  [-MASS_LINK_3*LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0])]] ) # Gravity matrix
def get_jacobian(q):
    return np.array([[-(LENGTH_LINK_1*np.sin(q[0,0]) + LENGTH_LINK_2 * np.sin(q[0,0]+q[1,0]) + LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0])),
                 -(LENGTH_LINK_2*np.sin(q[0,0]+q[1,0]) + LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0])), -LENGTH_LINK_3*np.sin(q[0,0]+q[1,0]+q[2,0]),
                   ], [-(LENGTH_LINK_1*np.cos(q[0,0]) + LENGTH_LINK_2 * np.cos(q[0,0]+q[1,0]) + LENGTH_LINK_3*np.cos(q[0,0]+q[1,0]+q[2,0])),
                 -(LENGTH_LINK_2*np.cos(q[0,0]+q[1,0]) + LENGTH_LINK_3*np.cos(q[0,0]+q[1,0]+q[2,0])), -LENGTH_LINK_3*np.cos(q[0,0]+q[1,0]+q[2,0]),
                   ], [0,0,0]])