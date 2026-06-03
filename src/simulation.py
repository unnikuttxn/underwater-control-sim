import numpy as np
from .controller import PIDController
from .plotting import plot_results

def run_simulation():
    m=50.0
    cd=20.0
    desired_depth=10.0

    dt=0.01
    t_final=50

    time=np.arange(0,t_final+dt,dt)
    N=len(time)

    z=0.0
    v=0.0

    pid=PIDController(80,1,30)

    depth_history=np.zeros(N)
    velocity_history=np.zeros(N)
    thrust_history=np.zeros(N)
    error_history=np.zeros(N)

    for k in range(N):
        error=desired_depth-z
        thrust=pid.update(error,dt)

        current_force=20*np.sin(0.5*time[k])
        drag=cd*v
        acceleration=(thrust-drag+current_force)/m

        v=v+acceleration*dt
        z=z+v*dt

        depth_history[k]=z
        velocity_history[k]=v
        thrust_history[k]=thrust
        error_history[k]=error

    overshoot=((np.max(depth_history)-desired_depth)/desired_depth)*100
    steady_state_error=abs(desired_depth-depth_history[-1])

    print(f'Maximum Depth: {np.max(depth_history):.2f} m')
    print(f'Overshoot: {overshoot:.2f}%')
    print(f'Final Depth: {depth_history[-1]:.2f} m')
    print(f'Steady-State Error: {steady_state_error:.4f} m')

    plot_results(time, depth_history, velocity_history,
                 thrust_history, error_history, desired_depth)
