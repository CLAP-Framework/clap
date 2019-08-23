cimport cython

import math
import numpy as np
cimport numpy as np
import numpy.linalg as npl

from shapely.geometry import Polygon

cpdef dist_from_point_to_line2d(float x0, float y0, float x1, float y1, float x2, float y2):
    '''
    Calculate distance to the endpoint line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
    Return: distance_to_line, point1_to_foot_point, foot_point_to_point2, the value is signed (i.e. minus if obtuse angle)
    Note: the distance is negative if the point is at left hand side of the direction of line (p1 -> p2)
    '''
    cdef float l = math.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))
    cdef float dl = ((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - x1*y2) / l
    cdef float d1 = (x1*x1+x0*(x2-x1)-x1*x2 + y1*y1+y0*(y2-y1)-y1*y2) / l
    cdef float d2 = (x2*x2-x0*(x2-x1)-x1*x2 + y2*y2-y0*(y2-y1)-y1*y2) / l
    
    return dl, d1, d2

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef dist_from_point_to_polyline2d(float x0, float y0, np.ndarray line, bint return_end_distance=False):
    """
    line: In shape of Nx2 point array
    Return: distance_to_line, closest_point_index
        (if return_end_distance: line_head_to_foot_point, foot_point_to_line_end)
    Note: all returned distance are signed
    """

    cdef np.ndarray dist_pc = npl.norm(line - [x0, y0], axis=1) # dist from current point (x0, y0) to line points
    cdef int idx_pc = np.argmin(dist_pc) # index of closet point
    
    cdef float distp_pc, distp_pp, distp_pn, distn_pc, distn_pp, distn_pn
    distp_pc = distp_pp = distp_pn = float('inf') # distance from point to previous line, previous line header, previous line end
    distn_pc = distn_pp = distn_pn = float('inf') # distance from point to next line, next line header, next line end

    # Calculate distances mentioned above
    if idx_pc != 0:
        distp_pc, distp_pp, distp_pn = dist_from_point_to_line2d(x0, y0,
            line[idx_pc-1, 0], line[idx_pc-1, 1],
            line[idx_pc, 0], line[idx_pc, 1])
    if idx_pc != len(line) - 1:
        distn_pc, distn_pp, distn_pn = dist_from_point_to_line2d(x0, y0,
            line[idx_pc, 0], line[idx_pc, 1],
            line[idx_pc+1, 0], line[idx_pc+1, 1])

    # Return early if total distance is not needed
    cdef float dist_pline = min(distp_pc, distn_pc)
    if not return_end_distance:
        return (dist_pline, idx_pc)

    cdef float dist_pstart, dist_pend
    if distp_pc > distn_pc:
        # closer to next line segment
        dist_pstart = distn_pp + np.sum(npl.norm(np.diff(line[:idx_pc+1], axis=0), axis=1))
        dist_pend = distn_pn + np.sum(npl.norm(np.diff(line[idx_pc+1:], axis=0), axis=1))
    else:
        # closer to previous line segment
        dist_pstart = distp_pp + np.sum(npl.norm(np.diff(line[:idx_pc], axis=0), axis=1))
        dist_pend = distp_pn + np.sum(npl.norm(np.diff(line[idx_pc:], axis=0), axis=1))
    
    # If the point is away from polyline area, dist_pstart and dist_pend will be negative
    return dist_pline, idx_pc, dist_pstart, dist_pend

def dense_polyline2d(np.ndarray line, float resolution, str interp="linear"):
    """
    Dense a polyline by linear interpolation.

    resolution: The gap between each point <= resolution
    interp: The interpolation method

    Return: The densed polyline 
    """
    if line is None or len(line) == 0:
        raise ValueError("Line input is null")

    if interp != "linear":
        raise NotImplementedError("Other interpolation method is not implemented!")

    cdef np.ndarray s
    s = np.cumsum(npl.norm(np.diff(line, axis=0), axis=1))
    s = np.concatenate([[0],s])
    cdef int num = math.ceil(s[-1]/resolution)

    cdef np.ndarray s_space = np.linspace(0,s[-1],num = num)
    cdef np.ndarray x = np.interp(s_space,s,line[:,0])
    cdef np.ndarray y = np.interp(s_space,s,line[:,1])

    return np.array([x,y]).T

def polygon_iou(p1, p2):
    """
    Intersection area / Union area of two polygons
    """
    p1, p2 = Polygon(p1), Polygon(p2)
    pi = p1.intersection(p2).area
    pu = p1.area + p2.area - pi
    return pi / pu

def box_to_corners_2d(xy, wh, yaw):
    """
    Convert a oriented box to corners on 2d plane.
    Returned box is in counterclock order
    """
    rot_yaw = np.array([
        [ np.cos(yaw), np.sin(yaw)],
        [-np.sin(yaw), np.cos(yaw)]
    ])
    x, y = xy
    dx, dy = np.dot(rot_yaw, np.reshape(wh, (-1,1)) / 2)
    return [(x+dx, y+dy), (x-dx, y+dy), (x-dx, y-dy), (x+dx, y-dy)]
