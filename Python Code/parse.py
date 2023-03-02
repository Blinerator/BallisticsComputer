import re

def p(x):
    x=re.sub("[^\d\.]","",x)
    x=float(x)
    return x