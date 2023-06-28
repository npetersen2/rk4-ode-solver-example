# Straight copied from Wikipedia:
#
# https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method


def solver_rk4_step(dy_dt, y_n, t_n, h):
    k1 = dy_dt(t_n, y_n)
    k2 = dy_dt(t_n + h / 2, y_n + h * k1 / 2)
    k3 = dy_dt(t_n + h / 2, y_n + h * k2 / 2)
    k4 = dy_dt(t_n + h, y_n + h * k3)

    y_next = y_n + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    t_next = t_n + h

    return (y_next, t_next)
