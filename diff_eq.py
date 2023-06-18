from applied_voltage import applied_voltage

L = 40e-3
R = 2000e-3


def dy_dt(t, i):
    return (1 / L) * applied_voltage(t) - (R / L) * i
