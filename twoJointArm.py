from operator import truediv

import matplotlib.pyplot as plt
import numpy as np
import math
from utils.angle import angle_mod
from scipy.interpolate import make_interp_spline

show_animation = True

fig = plt.figure()
point = []

# Setting lenth of arms
l1 = 1
l2 = 0.5

dt = 0.01

if show_animation:
    plt.ion()

def two_joint_arm(path_points):
    for pt in path_points:
        x, y = pt
        r = np.hypot(x, y)
        theta_dir = math.atan2(y, x)
        r_min = abs(l1 - l2)
        r_max = l1 + l2
        if r < r_min:
            x = r_min * math.cos(theta_dir)
            y = r_min * math.sin(theta_dir)
        elif r > r_max:
            x = r_max * math.cos(theta_dir)
            y = r_max * math.sin(theta_dir)

        # Get the theta2 first
        # theta2 = 180 - beta
        beta = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
        beta = np.clip(beta, -1.0, 1.0)
        theta2 = math.acos(beta)

        # Calculate alpha with theta2
        # theta1 = tan(x/y) - alpha
        alpha = math.atan2(l2*math.sin(theta2), l1 + l2*math.cos(theta2))
        theta1 = math.atan2(y, x) - alpha

        if theta1 < 0:
            theta2 = -theta2
            alpha = math.atan2(l2*math.sin(theta2), l1 + l2*math.cos(theta2))
            theta1 = math.atan2(y, x) - alpha

        x_actual = l1*math.cos(theta1) + l2*math.cos(theta1+theta2)
        y_actual = l1*math.sin(theta1) + l2*math.sin(theta1+theta2)

        plot_arm(theta1, theta2, x_actual, y_actual)


# Refer Start ## Yuxiang Ding
# https://github.com/AtsushiSakai/PythonRobotics.git
def plot_arm(theta1, theta2, target_x, target_y):  # pragma: no cover
    shoulder = np.array([0, 0])
    elbow = shoulder + np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    wrist = elbow + \
        np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])

    if show_animation:
        plt.cla()

        circle_out = plt.Circle((0, 0), l1 + l2, color='lightgray', fill=False, linestyle='-')
        plt.gca().add_patch(circle_out)

        circle_inner = plt.Circle((0, 0), l1 - l2, color='lightgray', fill=False, linestyle='-')
        plt.gca().add_patch(circle_inner)

        t = np.arange(len(point))
        x_vals, y_vals = zip(*point)

        tnew = np.linspace(t[0], t[-1], 200)

        spl_x = make_interp_spline(t, x_vals, k=3)
        spl_y = make_interp_spline(t, y_vals, k=3)

        xnew = spl_x(tnew)
        ynew = spl_y(tnew)

        plt.plot(xnew, ynew, 'g-', linewidth=2)

        plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'k-')
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'k-')

        plt.plot(shoulder[0], shoulder[1], 'ro')
        plt.plot(elbow[0], elbow[1], 'ro')
        plt.plot(wrist[0], wrist[1], 'ro')

        plt.plot([wrist[0], target_x], [wrist[1], target_y], 'g--')
        plt.plot(target_x, target_y, 'g*')

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.show()
        plt.pause(dt)

    return wrist
# Refer End ## Yuxiang Ding

def ang_diff(theta1, theta2):
    # Returns the difference between two angles in the range -pi to +pi
    return angle_mod(theta1 - theta2)

def click(event):
    global x, y
    x = event.xdata
    y = event.ydata
    point.append([x, y])
    plt.plot(x, y, 'ro')
    fig.canvas.draw()

    if len(point) == 5:
        t = np.arange(len(point))
        x_vals, y_vals = zip(*point)

        tnew = np.linspace(t[0], t[-1], 200)

        spl_x = make_interp_spline(t, x_vals, k=3)
        spl_y = make_interp_spline(t, y_vals, k=3)

        xnew = spl_x(tnew)
        ynew = spl_y(tnew)

        path_points = np.column_stack((xnew, ynew))

        two_joint_arm(path_points)

def main():  # pragma: no cover
    fig.canvas.mpl_connect("button_press_event", click)
    # fig.canvas.mpl_connect("button_release_event", click_done)

    plt.cla()
    plt.axis([-1.5, 1.5, -1.5, 1.5])

    circle_out = plt.Circle((0, 0), l1 + l2, color='lightgray', fill=False, linestyle='-')
    plt.gca().add_patch(circle_out)

    circle_inner = plt.Circle((0, 0), l1 - l2, color='lightgray', fill=False, linestyle='-')
    plt.gca().add_patch(circle_inner)

    plt.axis('equal')

    plt.show()
    plt.pause(10)

if __name__ == "__main__":
    # animation()
    main()
