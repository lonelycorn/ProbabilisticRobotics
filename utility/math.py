import numpy as np


def wrap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def angular_velocity_is_trivial(omega):
    '''
    test if the provided angular velocity is small enough to be treated as zero
    '''
    return (omega < 1e-5)


if __name__ == '__main__':
    pass
