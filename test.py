import numpy as np

def roundDown(x, decimalPlaces):
    y = 10**(-decimalPlaces)
    return np.round(np.floor(x/y)*y,decimalPlaces)

print(roundDown(5.1923, 1))