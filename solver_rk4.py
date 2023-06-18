from diff_eq import dy_dt


def solver_rk4_step(y_n, t_n, h):
    k1 = dy_dt(t_n, y_n)
    k2 = dy_dt(t_n + h / 2, y_n + h * k1 / 2)
    k3 = dy_dt(t_n + h / 2, y_n + h * k2 / 2)
    k4 = dy_dt(t_n + h, y_n + h * k3)

    y_next = y_n + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    t_next = t_n + h

    return (y_next, t_next)
