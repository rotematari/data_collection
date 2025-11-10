import random
import math
import plotly.graph_objects as go

def sample_positions(n, goal_x):
    x_fixed = 0.595
    y_fixed = -0.295
    z_fixed = 0.173
    
    x_start = 0.105
    y_start = -0.295
    z_start = 0.173
    wire_length = 0.49
    
    x_min = x_start + 0.08
    x_max = x_fixed - 0.1

    samples = []
    for _ in range(n):
        x = random.uniform(x_min, x_max)
        y_sacl = random.uniform(-1, 1)
        
        
        y = math.sqrt(wire_length**2 - (x_fixed - x)**2)*y_sacl + y_fixed
        z = z_fixed
        
        samples.append((x, y, z))
    return samples


def visualize(samples):
    xs, ys, zs = zip(*samples)
    x_fixed, y_fixed, z_fixed = 0.595, -0.295, 0.173
    # start pose
    x_start, y_start, z_start = 0.105, -0.295, 0.173

    fig = go.Figure()

    # samples
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="markers",
        marker=dict(size=4, color="blue"),
        name="Samples"
    ))

    # fixed endpoint
    fig.add_trace(go.Scatter3d(
        x=[x_fixed], y=[y_fixed], z=[z_fixed],
        mode="markers",
        marker=dict(size=8, color="red", symbol="x"),
        name="Fixed point"
    ))
    # start point
    fig.add_trace(go.Scatter3d(
        x=[x_start], y=[y_start], z=[z_start],
        mode="markers",
        marker=dict(size=8, color="green", symbol="circle"),
        name="Start point"
    ))

    # styling
    fig.update_layout(
        scene=dict(
            xaxis_title="x",
            yaxis_title="y",
            zaxis_title="z",
            aspectmode="data"
        ),
        title="Sampled wire endpoints",
    )
    fig.show()


if __name__ == "__main__":
    n = 1000
    goal_x = 0.105
    samples = sample_positions(n, goal_x)
    visualize(samples)
