#several lines to handle parsing LRF reading.
#Could have been put in main.py but was cleaner this way
import re
def p(x):
    x=re.sub("[^\d\.]","",x)
    x=float(x)
    return x
