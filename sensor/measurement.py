from numpy import rad2deg
from numpy import arctan2, sin, cos

class Measurement:
    '''
    This class models the measurement for a range sensor.
    Every measurement is made up of three fields, a distance r, a heading phi
    and a signature s.
    A measurement should be interpreted as, "A landmark whose signature is s
    is observed at distance r in the relative direction phi".
    '''
    def __init__(self, r, phi, s):
        '''
        The constructor.
        ===INPUT===
        r:      the relative distance in meters.
        phi:    the relative bearing in radians.
        s:      the signature observed.
        ===OUTPUT===
        none
        '''
        self.r = r
        self.phi = arctan2(sin(phi), cos(phi))
        self.s = s

    def __str__(self):
        return 'Measurement: r = ' + str(self.r) + ' m, phi = ' + \
                str(rad2deg(self.phi)) + ' deg, s = ' + str(self.s)
