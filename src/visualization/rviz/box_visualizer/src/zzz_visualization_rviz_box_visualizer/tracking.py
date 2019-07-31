from easydict import EasyDict
import rospy
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from zzz_visualization_rviz_box_visualizer.utils import parse_color

class TrackingBoxVisualizer:
    def __init__(self, **params):
        self._marker_id = 0 # XXX: Do we need to hold same ID for markers of same object?
        self._params = EasyDict(params)
        
        if isinstance(self._params.label_color, list):
            self._params.label_color = parse_color(self._params.label_color)
        if isinstance(self._params.box_color, list):
            self._params.box_color = parse_color(self._params.box_color)
        if isinstance(self._params.centroid_color, list):
            self._params.centroid_color = parse_color(self._params.centroid_color)
        if isinstance(self._params.arrow_color, list):
            self._params.arrow_color = parse_color(self._params.arrow_color)

    def addLabels(self, in_objects, out_markers):
        for obj in in_objects.targets:
            label_marker = Marker()
            label_marker.lifetime = rospy.Duration(self._params.marker_lifetime)

            # Message headers
            label_marker.header = in_objects.header
            label_marker.action = Marker.ADD
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.ns = self._params.marker_namespace + "/labels"
            label_marker.id = self._marker_id
            self._marker_id += 1

            # Marker properties
            label_marker.scale.x = self._params.label_scale
            label_marker.scale.y = self._params.label_scale
            label_marker.scale.z = self._params.label_scale
            label_marker.color = self._params.label_color

            label_marker.text = "#%d: " % obj.uid
            if len(obj.classes) > 0: # Use classification name if presented
                label_marker.text += "[" + str(obj.classes[0].classid) + "]"

            x = obj.bbox.pose.pose.position.x
            y = obj.bbox.pose.pose.position.y
            z = obj.bbox.pose.pose.position.z
            vx = obj.twist.twist.linear.x
            vy = obj.twist.twist.linear.y
            vz = obj.twist.twist.linear.z
            label_marker.text += "%.2f m, %.2f m/s" % (np.sqrt(x*x + y*y + z*z), np.sqrt(vx*vx + vy*vy + vz*vz))
            if obj.comments:
                label_marker.text += "\n" + obj.comments

            label_marker.pose.position.x = x
            label_marker.pose.position.y = y
            label_marker.pose.position.z = self._params.label_height
            
            out_markers.markers.append(label_marker)

    def addBoxes(self, in_objects, out_markers):
        for obj in in_objects.targets:
            box = Marker()
            box.lifetime = rospy.Duration(self._params.marker_lifetime)

            # Message headers
            box.header = in_objects.header
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.ns = self._params.marker_namespace + "/boxs"
            box.id = self._marker_id
            self._marker_id += 1

            # Marker properties
            box.color = self._params.box_color

            box.scale.x = obj.bbox.dimension.length_x
            box.scale.y = obj.bbox.dimension.length_y
            box.scale.z = obj.bbox.dimension.length_z
            box.pose.position = obj.bbox.pose.pose.position
            box.pose.orientation = obj.bbox.pose.pose.orientation

            # Skip large boxes to prevent messy visualization
            if box.scale.x > self._params.box_max_size or box.scale.y > self._params.box_max_size or box.scale.z > self._params.box_max_size:
                rospy.logdebug("Object skipped with size %.2f x %.2f x %.2f!",
                    box.scale.x, box.scale.y, box.scale.z)
                continue

            box_str = str(box)
            if 'nan' in box_str or 'inf' in box_str:
                print("----------------------------------")
                print(box_str)
            out_markers.markers.append(box)

    def addCentroids(self, in_objects, out_markers):
        for obj in in_objects.targets:
            centroid_marker = Marker()
            centroid_marker.lifetime = rospy.Duration(self._params.marker_lifetime)

            # Message headers
            centroid_marker.header = in_objects.header
            centroid_marker.type = Marker.SPHERE
            centroid_marker.action = Marker.ADD
            centroid_marker.ns = self._params.marker_namespace + "/centroids"
            centroid_marker.id = self._marker_id
            self._marker_id += 1

            # Marker properties
            centroid_marker.scale.x = self._params.centroid_scale
            centroid_marker.scale.y = self._params.centroid_scale
            centroid_marker.scale.z = self._params.centroid_scale
            centroid_marker.color = self._params.centroid_color

            centroid_marker.pose = obj.bbox.pose.pose

            out_markers.markers.append(centroid_marker)

    def addArrow(self, in_objects, out_markers):
        '''
        This function shows the velocity direction of the object
        '''
        for obj in in_objects.targets:
            arrow_marker = Marker()
            arrow_marker.lifetime = rospy.Duration(self._params.marker_lifetime)

            # Message headers
            arrow_marker.header = in_objects.header
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.ns = self._params.marker_namespace + "/arrows"
            arrow_marker.id = self._marker_id
            self._marker_id += 1

            # Marker properties
            arrow_marker.scale.y = self._params.arrow_width
            arrow_marker.scale.z = self._params.arrow_width
            arrow_marker.color = self._params.arrow_color

            arrow_marker.pose.position = obj.bbox.pose.pose.position

            vel = np.array([obj.twist.twist.linear.x, obj.twist.twist.linear.y, obj.twist.twist.linear.z])
            vel_len = np.linalg.norm(vel)
            if vel_len > 0:
                arrow_marker.scale.x = vel_len * self._params.arrow_speed_scale
                vel = vel / vel_len
                rot = np.cross([1,0,0], vel)
                angle_sin = np.dot([1,0,0], vel)
                angle_cos = np.linalg.norm(rot)
                angle = np.arctan2(angle_sin, angle_cos)

                if angle != 0:
                    obj.bbox.pose.pose.orientation.x = rot[0] / np.sin(angle/2)
                    obj.bbox.pose.pose.orientation.y = rot[1] / np.sin(angle/2)
                    obj.bbox.pose.pose.orientation.z = rot[2] / np.sin(angle/2)
                    obj.bbox.pose.pose.orientation.w = np.cos(angle/2)
            else:
                # No arrow is shown if the velocity is zero
                continue

            out_markers.markers.append(arrow_marker)

    def visualize(self, msg):
        markers = MarkerArray()
        self._marker_id = 0

        self.addBoxes(msg, markers)
        self.addCentroids(msg, markers)
        self.addLabels(msg, markers)
        self.addArrow(msg, markers)

        return markers        
