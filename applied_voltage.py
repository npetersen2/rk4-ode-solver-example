import numpy as np

# Generate a voltage profile as a function of time
#
# Implements 5 Hz square wave with peak of 1 V


def applied_voltage(t):
    Vpk = 1
    v = Vpk * np.array([1, 0, 1, 0, 1, 0, 1, 0, 1, 0])

    # how long to hold each voltage level
    hold_ms = 100

    v_val = np.repeat(v, hold_ms)
    t_end = (hold_ms * len(v)) * 1e-3
    t_val = np.linspace(0, t_end - 1e-3, hold_ms * len(v))

    # Assume t is in seconds
    return np.interp(t, t_val, v_val)
