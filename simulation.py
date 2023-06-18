from solver_rk4 import solver_rk4_step
from applied_voltage import applied_voltage
import numpy as np
import matplotlib.pyplot as plt

# Step size
step_size = 2e-3  # sec

# Simulation end time
t_end = 0.2  # sec

# Time points to solve the simulation
t_vec = np.arange(0, t_end, step_size)

# Initial conditions
i_0 = 0.0


def main():
    # Time at this time step
    t_n = t_vec[0]

    # Current at this time step
    i_n = i_0

    # Initial state
    i_out = [i_n]
    v_applied = []

    # This loop solves for the NEXT state based on the current state
    for _ in range(0, len(t_vec)):
        v_n = applied_voltage(t_n)
        v_applied.append(v_n)

        # display state to console
        print(t_n, v_n, i_n)

        (i_next, t_next) = solver_rk4_step(i_n, t_n, step_size)
        i_out.append(i_next)

        i_n = i_next
        t_n = t_next

    fig, ax = plt.subplots(1, 1)

    ax.plot(t_vec, v_applied, label="Voltage")
    ax.plot(t_vec, i_out[0:-1], label="Current", marker=".")

    ax.set_xlabel("Time (sec)")

    ax.legend(loc="upper right")

    fig.set_size_inches(4, 3)
    fig.tight_layout()
    fig.savefig("plot_sim.png", dpi=300)


if __name__ == "__main__":
    main()
