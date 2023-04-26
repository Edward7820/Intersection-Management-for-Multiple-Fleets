import math

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
