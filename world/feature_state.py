class FeatureState:
    def __init__(self, x, y, s):
        self.x = x * 1.0
        self.y = y * 1.0
        self.s = s # signature

    def __str__(self):
        return 'FeatureState: pos = (' + str(self.x) + ', ' + str(self.y) + '), s = ' + str(self.s)

if __name__ == "__main__":
    pass
