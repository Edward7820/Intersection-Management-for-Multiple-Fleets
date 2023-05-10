import math
from typing import Tuple

def vector_length(x: float, y: float):
    return math.sqrt(x**2+y**2)

def quadratic(a:float, b:float, c:float):
    #solve ax^2+bx+c=0 and return the bigger root
    if a==0:
        return -c/b
    key = b**2-4*a*c
    root = None
    if key>0:
        root=(-b+math.sqrt(key))/(2*a)
    elif key==0:
        root=-b/(2*a)
    return root

def euclidean_dist(x1: Tuple[float], x2: Tuple[float]):
    return math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)
