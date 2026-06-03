import numpy as np

from .controller import PIDController
from .plotting import plot_results


def run_simulation():
    """
    AUV Depth Control Simulation
    Includes:
    - PID controller
    - Buoyancy
    - Gravity
    - Added mass
    - Quadratic hydrodynamic drag
    - Thruster saturation
    - Ocean current disturbance
    """

    mass = 50.0                 # kg
    added_mass = 15.0           # kg

    g = 9.81                    # m/s²

    rho_water = 1025.0          # kg/m³
    vehicle_volume = 0.050      # m³

    drag_coefficient = 35.0

    # Thruster limits
    max_thrust = 400.0          # N
    min_thrust = -400.0         # N

    buoyancy_force = (
        rho_water *
        vehicle_volume *
        g
    )

    weight_force = (
        mass *
        g
    )
    desired_depth = 10.0

    dt = 0.01
    t_final = 50.0

    time = np.arange(
        0,
        t_final + dt,
        dt
    )

    N = len(time)

    depth = 0.0
    velocity = 0.0

    pid = PIDController(
        kp=80,
        ki=1,
        kd=30
    )

    depth_history = np.zeros(N)
    velocity_history = np.zeros(N)
    thrust_history = np.zeros(N)
    error_history = np.zeros(N)
    
    for k in range(N):

        error = desired_depth - depth

        thrust = pid.update(
            error,
            dt
        )

        # Thruster saturation
        thrust = np.clip(
            thrust,
            min_thrust,
            max_thrust
        )

        # Ocean current disturbance
        current_force = (
            20 *
            np.sin(
                0.5 * time[k]
            )
        )

        # Quadratic drag
        drag_force = (
            drag_coefficient *
            abs(velocity) *
            velocity
        )

        # Effective mass
        effective_mass = (
            mass +
            added_mass
        )

        acceleration = (
            thrust
            + buoyancy_force
            - weight_force
            - drag_force
            + current_force
        ) / effective_mass

        velocity += acceleration * dt
        depth += velocity * dt

        depth_history[k] = depth
        velocity_history[k] = velocity
        thrust_history[k] = thrust
        error_history[k] = error
        
    overshoot = (
        (
            np.max(depth_history)
            - desired_depth
        )
        / desired_depth
    ) * 100

    steady_state_error = abs(
        desired_depth
        - depth_history[-1]
    )

    iae = np.sum(
        np.abs(error_history)
    ) * dt

    rmse = np.sqrt(
        np.mean(
            error_history**2
        )
    )

    print("\n===== PERFORMANCE METRICS =====")

    print(
        f"Maximum Depth: "
        f"{np.max(depth_history):.2f} m"
    )

    print(
        f"Overshoot: "
        f"{overshoot:.2f}%"
    )

    print(
        f"Final Depth: "
        f"{depth_history[-1]:.2f} m"
    )

    print(
        f"Steady-State Error: "
        f"{steady_state_error:.4f} m"
    )

    print(
        f"IAE: "
        f"{iae:.4f}"
    )

    print(
        f"RMSE: "
        f"{rmse:.4f}"
    )

    plot_results(
        time,
        depth_history,
        velocity_history,
        thrust_history,
        error_history,
        desired_depth
    )
