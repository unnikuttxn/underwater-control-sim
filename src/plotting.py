import matplotlib.pyplot as plt

def plot_results(time, depth, velocity, thrust, error, desired_depth):
    plt.figure(figsize=(10,8))

    plt.subplot(2,2,1)
    plt.plot(time, depth, linewidth=2)
    plt.axhline(desired_depth, linestyle='--')
    plt.title('Depth Response')
    plt.grid(True)

    plt.subplot(2,2,2)
    plt.plot(time, velocity, linewidth=2)
    plt.title('Vertical Velocity')
    plt.grid(True)

    plt.subplot(2,2,3)
    plt.plot(time, thrust, linewidth=2)
    plt.title('PID Controller Output')
    plt.grid(True)

    plt.subplot(2,2,4)
    plt.plot(time, error, linewidth=2)
    plt.title('Tracking Error')
    plt.grid(True)

    plt.tight_layout()
    plt.show()
