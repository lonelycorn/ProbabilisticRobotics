import numpy as np

INFINITY = 1E5
EPSILON = 1E-5

def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def angular_velocity_is_trivial(omega):
    '''
    test if the provided angular velocity is small enough to be treated as zero
    '''
    return (np.abs(omega) < EPSILON)


if __name__ == '__main__':
    pass
