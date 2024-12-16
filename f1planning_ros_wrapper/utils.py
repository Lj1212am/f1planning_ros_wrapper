# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Utility functions for Kinematic Single Track MPC waypoint tracker

Author: Hongrui Zheng, Johannes Betz, Ahmad Amine
Last Modified: 12/27/22
"""
import math
import numpy as np
# from numba import njit

# @njit(cache=True)
def nearest_point(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    Args:
        point (numpy.ndarray, (2, )): (x, y) of current pose
        trajectory (numpy.ndarray, (N, 2)): array of (x, y) trajectory waypoints
            NOTE: points in trajectory must be unique. If they are not unique, a divide by 0 error will destroy the world
    Returns:
        nearest_point (numpy.ndarray, (2, )): nearest point on the trajectory to the point
        nearest_dist (float): distance to the nearest point
        t (float): nearest point's location as a segment between 0 and 1 on the vector formed by the closest two points on the trajectory. (p_i---*-------p_i+1)
        i (int): index of nearest point in the array of trajectory waypoints
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    projections = trajectory[:-1,:] + (t*diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment


def transform_track(x, y, angle, offset_x=0.0, offset_y=0.0):
    """
    Rotates the track around its own geometric center and then optionally shifts it.
    
    Parameters
    ----------
    x : np.ndarray
        The x-coordinates of the track.
    y : np.ndarray
        The y-coordinates of the track.
    angle : float
        The angle in radians by which to rotate the track around its center.
    offset_x : float, optional
        Additional translation in the x-direction after rotation. Default is 0.0.
    offset_y : float, optional
        Additional translation in the y-direction after rotation. Default is 0.0.

    Returns
    -------
    x_transformed : np.ndarray
        The transformed x-coordinates.
    y_transformed : np.ndarray
        The transformed y-coordinates.
    """
    # Compute the center of the track
    center_x = (x.min() + x.max()) / 2.0
    center_y = (y.min() + y.max()) / 2.0

    # Translate so the center is at the origin
    x_shifted = x - center_x
    y_shifted = y - center_y

    # Rotation
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    x_rotated = x_shifted * cos_a - y_shifted * sin_a
    y_rotated = x_shifted * sin_a + y_shifted * cos_a

    # Translate back from origin and then apply the user-specified offsets
    x_transformed = x_rotated + center_x + offset_x
    y_transformed = y_rotated + center_y + offset_y

    return x_transformed, y_transformed