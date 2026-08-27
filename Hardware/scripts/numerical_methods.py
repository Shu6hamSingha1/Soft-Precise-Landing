# Numerical Methods
import numpy as np

def RK5(func, t=0, y_0=0, u=None, h=0.001):
    F1 = func(t     ,y_0    , u)
    F2 = func(t+h/4    , y_0+(h/4)*F1, u)
    F3 = func(t+h/4    , y_0+(h/8)*F1+(h/8)*F2, u)
    F4 = func(t+h/2    , y_0-(h/2)*F2+h*F3, u)
    F5 = func(t+(h*3/4), y_0+(h*3/16)*F1+(h*9/16)*F4, u)
    F6 = func(t+h      , y_0-(h*3/7)*F1+(h*2/7)*F2+(h*12/7)*F3-(h*12/7)*F4+(h*8/7)*F5, u)
    y = y_0 + (h/90)*(7*F1+32*F3+12*F4+32*F5+7*F6)
    return y

def RK4(func, t, y_0, h):
    k1 = func(t    , y_0         )
    k2 = func(t+h/2, y_0+(h/2)*k1)
    k3 = func(t+h/2, y_0+(h/2)*k2)
    k4 = func(t+h  , y_0+h*k3    )
    y = y_0 + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
    return y

def smooth4(data):    
    return (0.4*data[-1] + 0.3*data[-2] + 0.2*data[-3] + 0.1*data[-4])

def extrapolate(t, y, n=4, deg=1, default_shape=6):
    if len(t) < n+1 or len(y) < n:
        if len(y) == 0:
            return np.zeros(default_shape)  # Default shape for optical flow; adjust if needed (e.g., 4 for img features)
        return np.mean(y, axis=0)
        
    # Fit polynomial to smoothed data
    coeffs = np.polyfit(t[-n:], y[-n:], deg)
    
    # Extrapolate one step forward
    return np.polyval(coeffs, t[-1])
