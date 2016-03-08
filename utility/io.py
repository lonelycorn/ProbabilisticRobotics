#import numpy as np

def print_matrix(m, title = None):
    s = m.shape
    if (title is not None):
        print("===== " + title + " =====")
    for i in range(s[0]):
        for j in range(s[1]):
            g = float("{0:.2f}".format(m[i, j]))
            print(g, end="    ")
        print(" ")

