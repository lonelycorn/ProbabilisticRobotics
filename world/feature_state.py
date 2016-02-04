class FeatureState:
    '''
    This class models the fundamental elements in feature-based maps.
    Every feature is defined by a pair of coordinates and a signature.
    '''
    def __init__(self, x, y, s):
        '''
        The constructor.
        ===INPUT===
        x: map feature's X, in meters
        y: map feature's Y, in meters
        s: map feature's signature.
        '''
        self.x = x * 1.0
        self.y = y * 1.0
        self.s = s

    def __str__(self):
        return 'FeatureState: pos = (' + str(self.x) + ', ' + str(self.y) + \
               '), s = ' + str(self.s)

if __name__ == "__main__":
    pass
