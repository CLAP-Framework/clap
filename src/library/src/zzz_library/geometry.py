
def dist_from_point_to_line(x0, y0, x1, y1, x2, y2):
    # Calculate distance to the endpoint line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
    return abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - x1*y2) / np.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))

def dist_from_point_to_polyline(x0, y0, line):
    raise NotImplementedError()
