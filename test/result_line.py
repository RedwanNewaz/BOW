import numpy as np
import warnings
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd
from matplotlib import cm
from matplotlib.colors import Normalize
from fire import Fire
from matplotlib.collections import LineCollection


def colored_line(x, y, c, ax, **lc_kwargs):
    """
    Plot a line with a color specified along the line by a third value.

    It does this by creating a collection of line segments. Each line segment is
    made up of two straight lines each connecting the current (x, y) point to the
    midpoints of the lines connecting the current point with its two neighbors.
    This creates a smooth line with no gaps between the line segments.

    Parameters
    ----------
    x, y : array-like
        The horizontal and vertical coordinates of the data points.
    c : array-like
        The color values, which should be the same size as x and y.
    ax : Axes
        Axis object on which to plot the colored line.
    **lc_kwargs
        Any additional arguments to pass to matplotlib.collections.LineCollection
        constructor. This should not include the array keyword argument because
        that is set to the color argument. If provided, it will be overridden.

    Returns
    -------
    matplotlib.collections.LineCollection
        The generated line collection representing the colored line.
    """
    if "array" in lc_kwargs:
        warnings.warn('The provided "array" keyword argument will be overridden')

    # Default the capstyle to butt so that the line segments smoothly line up
    default_kwargs = {"capstyle": "butt"}
    default_kwargs.update(lc_kwargs)

    N = len(x)
    segments = []
    for i in range(1, N):
        line = ((x[i - 1], y[i - 1]), (x[i], y[i]))
        segments.append(line)
    segments = np.array(segments)

    lc = LineCollection(segments, **default_kwargs)
    lc.set_array(c)  # set the colors of each segment

    return ax.add_collection(lc)
def readConfig(filename):
    with open(filename) as file:
        data = yaml.safe_load(file)
    obstacles = np.array(data['obstacles'])
    obstacle_length = data['obstacle_length']
    robot_radius = data['robot_radius']
    return obstacles, obstacle_length, robot_radius
def readResults(filename):
    data = pd.read_csv(filename).to_numpy()
    return data

def main(env='env1.yaml', result='result.csv'):
    traj = readResults(result)
    obstacles, obstacle_length, robot_radius = readConfig(env)


    minoXY = obstacles[:, :2].min(axis=0) - 1.0
    maxoXY = obstacles[:, :2].max(axis=0) + 1.0
    mintXY = traj[:, :2].min(axis=0) - 1.0
    maxtXY = traj[:, :2].max(axis=0) + 1.0
    minXY = np.vstack((minoXY, mintXY)).min(axis=0)
    maxXY = np.vstack((maxoXY, maxtXY)).max(axis=0)
    print(minXY, maxXY)

    # Create a figure and plot the line on it
    fig1, ax = plt.subplots(figsize=(16,10))
    lines = colored_line(traj[:, 0], traj[:, 1], traj[:, 3], ax, linewidth=3.5, cmap="rainbow", alpha=0.8)
    fig1.colorbar(lines)  # add a color legend

    # add obstacles

    # obs_width = obs_height = (obstacle_length + robot_radius / 2 ) * 2
    # for obs in obstacles:
    #     center = (obs[0] - obs_width / 2., obs[1] - obs_height / 2.)
    #     rect = Rectangle(center, obs_width, obs_height, color='k')
    #     ax.add_patch(rect)

    obs_width = obs_height = (2 * obstacle_length + robot_radius / 2)
    for obs in obstacles:
        center = (obs[0] - obstacle_length, obs[1] - obstacle_length)
        rect = Rectangle(center, obs_width, obs_height, color='k')
        ax.add_patch(rect)


    plt.grid()
    plt.axis([minXY[0], maxXY[0], minXY[1], maxXY[1]])
    plt.show()

if __name__ == '__main__':
    Fire(main)


