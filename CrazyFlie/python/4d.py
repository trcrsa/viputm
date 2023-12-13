import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

np.set_printoptions(suppress=True)


def animate_paths(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    total_duration = 0
    lines = []
    for path in paths:
        total_duration = max(total_duration, max(path[:, 3]))
        (line,) = ax.plot([], [], "o-")
        lines.append(line)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Drone Path")

    def update(frame):
        for lnum, line in enumerate(lines):
            path = paths[lnum]
            x = path[:, 0]
            y = path[:, 1]
            z = path[:, 2]
            time = path[:, 3]
            t = frame

            idx = np.searchsorted(time, t)

            if idx >= len(path):
                continue

            if idx == 0:
                x_interp = x[idx]
                y_interp = y[idx]
                z_interp = z[idx]
            else:
                t_segment = (t - time[idx - 1]) / (time[idx] - time[idx - 1])
                x_interp = x[idx - 1] + (x[idx] - x[idx - 1]) * t_segment
                y_interp = y[idx - 1] + (y[idx] - y[idx - 1]) * t_segment
                z_interp = z[idx - 1] + (z[idx] - z[idx - 1]) * t_segment

            line.set_data([x_interp], [y_interp])
            line.set_3d_properties([z_interp])
        return lines

    _ = FuncAnimation(
        fig, update, frames=np.arange(0, total_duration, 0.05), interval=20, blit=True
    )
    plt.show()


def closest_points_4d(P1, line1_point2, P2, line2_point2):
    V1 = line1_point2 - P1
    V2 = line2_point2 - P2

    parameters = np.dot(np.linalg.pinv(np.column_stack((V1, V2))), P2 - P1)

    closest_point_line1 = P1 + parameters[0] * V1
    closest_point_line2 = P2 - parameters[1] * V2

    return closest_point_line1, closest_point_line2, parameters[0], -parameters[1]


def is_valid_path(p1, p2):
    max_speed = 2
    if min(np.diff(p1[:, 3])) < 0 or min(np.diff(p2[:, 3])) < 0:
        return False

    return True


def perturbe_path(p):
    return p + np.hstack((np.random.rand(len(p), 3) / 100, np.zeros((len(p), 1))))


def handle_conflicts(p1, p2):
    for i in range(len(p1) - 1):
        for j in range(len(p2) - 1):
            if p1[i, 3] > p2[j + 1, 3] + 1 or p2[j, 3] > p1[i + 1, 3] + 1:
                continue

            cl_p1, cl_p2, param1, param2 = closest_points_4d(
                p1[i], p1[i + 1], p2[j], p2[j + 1]
            )
            diff = cl_p1 - cl_p2
            if np.linalg.norm(diff) > 1:
                continue
            if param1 < -0.05 or param1 > 1.05 or param2 < -0.05 or param2 > 1.05:
                continue

            point_idx1 = None
            point_idx2 = None
            if param1 < 0.05:
                point_idx1 = i
            elif param1 > 0.95:
                point_idx1 = i + 1

            if param2 < 0.05:
                point_idx2 = j
            elif param2 > 0.95:
                point_idx2 = j + 1

            if point_idx1 is None or point_idx2 is None:
                diff = diff @ np.diag([2, 2, 0.1, 1])
                norm = np.linalg.norm(diff)

            if point_idx1 is None:
                p1 = np.insert(p1, i + 1, cl_p1 + diff / norm, axis=0)
            else:
                move = diff @ np.diag([0, 0, 0, 1 / abs(diff[3])])
                p1[point_idx1] += move

            if point_idx2 is None:
                p2 = np.insert(p2, j + 1, cl_p2 - diff / norm, axis=0)
            else:
                move = diff @ np.diag([0, 0, 0, 1 / abs(diff[3])])
                p2[point_idx2] -= move

            p1_end, p2_end = handle_conflicts(p1[i:], p2[j:])

            return np.vstack((p1[:i], p1_end)), np.vstack((p2[:j], p2_end))
    return p1, p2


def find_path_conflicts(p1, p2):
    equivalent_tolerances = [0.1, 0.1, 1, 1]
    undo_scale = np.diag(equivalent_tolerances)
    scale = np.linalg.inv(undo_scale)

    while True:
        random_p1 = perturbe_path(p1)
        random_p2 = perturbe_path(p2)

        better_p1, better_p2 = handle_conflicts(random_p1 @ scale, random_p2 @ scale)
        undo1, undo2 = better_p1 @ undo_scale, better_p2 @ undo_scale
        if is_valid_path(undo1, undo2):
            return undo1, undo2


p1 = np.array(
    [
        [0, 0, 0, 0],
        [0, 0, 1, 2],
        [1, 0, 1, 4],
        [1, 1, 1, 6],
        [0, 1, 1, 8],
        [0, 1, 0, 10],
    ]
)
p2 = np.array(
    [
        [1, 1, 0, 0],
        [1, 1, 0.5, 2],
        [1, 0, 0.5, 4],
        [0, 1, 0.5, 6],
        [0, 0, 0.5, 8],
        [0, 0, 0, 10],
    ]
)

# p1 = np.array([[0,0,0,0],[1,1,1,5]], dtype=float)
# p2 = np.array([[1,1,1,0],[0,0,0,2]], dtype=float)

# p1 = np.array([[0, 0, 0, 0], [0.5, 0.5, 0.5, 2.5], [1, 1, 1, 5]], dtype=float)
# p2 = np.array([[1, 1, 1, 1], [0, 0, 0, 6]], dtype=float)

# p1 = np.array([[1, 0, 0, 0], [0.5, 0.5, 0.5, 2.5], [0, 1, 1, 5]], dtype=float)
# p2 = np.array([[1, 1, 1, 0], [0, 0, 0, 5]], dtype=float)


p1, p2 = find_path_conflicts(p1, p2)

print(p1)
print()
print(p2)

animate_paths([p1, p2])


# plt.show()
