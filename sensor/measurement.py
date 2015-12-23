from numpy import rad2deg
from numpy import arctan2, sin, cos

class Measurement:
    def __init__(self, r, phi, s):
        '''
        constructor
        ===INPUT===
        r:      the relative distance in meters
        phi:    the relative bearing in radians
        s:      the signature observed
        ===OUTPUT===
        none
        '''
        self.r = r
        self.phi = arctan2(sin(phi), cos(phi))
        self.s = s

    def __str__(self):
        return 'Measurement: r = ' + str(self.r) + ' m, phi = ' + \
                str(rad2deg(self.phi)) + ' deg, s = ' + str(self.s)
