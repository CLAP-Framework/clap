import numpy as np
import numpy.linalg as npl
from shapely.geometry import Polygon

def dist_from_point_to_line(x0, y0, x1, y1, x2, y2):
    '''
    Calculate distance to the endpoint line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
    Return: distance_to_line, point1_to_foot_point, foot_point_to_point2, the value is signed (i.e. minus if obtuse angle)
    '''
    l = np.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))
    dl = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - x1*y2) / l
    d1 = (x1*x1+x0*(x2-x1)-x1*x2 + y1*y1+y0*(y2-y1)-y1*y2) / l
    d2 = (x2*x2-x0*(x2-x1)-x1*x2 + y2*y2-y0*(y2-y1)-y1*y2) / l
    
    return dl, d1, d2

def nearest_point_to_polyline(x0, y0, line):
    """
    line: In shape of Nx2 point array
    Return: distance_to_line, the_index_of_nearest_point
    """

    assert len(line) > 0
    dist_pc = npl.norm(line - [x0, y0], axis=1) # dist from point (x0, y0) to line points
    idx_pc = np.argmin(dist_pc) # index of closet point

    return dist_pc[idx_pc], idx_pc

def dist_from_point_to_polyline(x0, y0, line):
    """
    line: In shape of Nx2 point array
    Return: distance_to_line, line_head_to_foot_point, foot_point_to_line_end
    """

    dist_pc = npl.norm(line - [x0, y0], axis=1) # dist from point (x0, y0) to line points
    idx_pc = np.argmin(dist_pc) # index of closet point
    
    distp_pc = distp_pp = distp_pn = float('inf') # distance from point to previous line, previous line header, previous line end
    distn_pc = distn_pp = distn_pn = float('inf') # distance from point to next line, next line header, next line end

    if idx_pc != 0:
        distp_pc, distp_pp, distp_pn = dist_from_point_to_line(x0, y0,
            line[idx_pc-1, 0], line[idx_pc-1, 1],
            line[idx_pc, 0], line[idx_pc, 1])
    if idx_pc != len(line) - 1:
        distn_pc, distn_pp, distn_pn = dist_from_point_to_line(x0, y0,
            line[idx_pc, 0], line[idx_pc, 1],
            line[idx_pc+1, 0], line[idx_pc+1, 1])

    if distp_pc > distn_pc:
        # closer to next line segment
        dist_pline = distn_pc
        dist_pstart = distn_pp + np.sum(npl.norm(np.diff(line[:idx_pc+1], axis=0), axis=1))
        dist_pend = distn_pn + np.sum(npl.norm(np.diff(line[idx_pc+1:], axis=0), axis=1))
    else:
        # closer to previous line segment
        dist_pline = distp_pc
        dist_pstart = distp_pp + np.sum(npl.norm(np.diff(line[:idx_pc], axis=0), axis=1))
        dist_pend = distp_pn + np.sum(npl.norm(np.diff(line[idx_pc:], axis=0), axis=1))
    
    # If the point is away from polyline area, dist_pstart of dist_pend will be negative
    return dist_pline, abs(dist_pstart), abs(dist_pend)

def dense_polyline(line, resolution, interp="linear"):
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

    s = np.cumsum(npl.norm(np.diff(line, axis=0), axis=1))
    s = np.concatenate([[0],s])
    num = s[-1]/resolution

    s_space = np.linspace(0,s[-1],num = num)
    x = np.interp(s_space,s,line[:,0])
    y = np.interp(s_space,s,line[:,1])

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
