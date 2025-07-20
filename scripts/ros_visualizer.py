#!/usr/bin/env python3

import rospy
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import IntEnum

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import Header


class MarkerType(IntEnum):
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3
    LINE_STRIP = 4
    LINE_LIST = 5
    CUBE_LIST = 6
    SPHERE_LIST = 7
    POINTS = 8
    TEXT_VIEW_FACING = 9
    MESH_RESOURCE = 10
    TRIANGLE_LIST = 11


@dataclass
class VisStyle:
    """Visualization style configuration"""
    r: float = 1.0
    g: float = 0.0
    b: float = 0.0
    a: float = 1.0
    x: float = 0.05  # width/size
    y: float = 0.05
    z: float = 0.05
    
    def __post_init__(self):
        # Ensure values are in valid ranges
        self.r = max(0.0, min(1.0, self.r))
        self.g = max(0.0, min(1.0, self.g))
        self.b = max(0.0, min(1.0, self.b))
        self.a = max(0.0, min(1.0, self.a))


@dataclass
class VisType:
    """Visualization type configuration"""
    name_space: str
    marker_type: int
    style: VisStyle


class ROSVisualizer:
    """ROS visualizer for RViz markers"""
    
    # Default styles
    STYLE_CURVE = VisStyle(1.0, 0.6, 0.002, 1.0, 0.01, 0.01, 0.01)
    STYLE_MESH = VisStyle(0.357, 0.458, 0.710, 0.3, 0.005, 0.005, 0.005)
    STYLE_FACET = VisStyle(0.15, 0.6, 0.8, 0.1, 1.0, 1.0, 1.0)
    STYLE_SCATTER = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02, 0.02, 0.02)
    STYLE_SPHERE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02, 0.02, 0.02)
    STYLE_CUBE = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02, 0.02, 0.02)
    STYLE_ARROW = VisStyle(1.0, 0.45, 0.0, 1.0, 0.02, 0.02, 0.02)
    STYLE_ARROW2 = VisStyle(0.8, 0.45, 0.8, 1.0, 0.04, 0.04, 0.04)
    
    # Default types
    TYPE_CURVE = VisType("curve", Marker.LINE_STRIP, STYLE_CURVE)
    TYPE_FACET = VisType("facet", Marker.TRIANGLE_LIST, STYLE_FACET)
    TYPE_MESH = VisType("mesh", Marker.LINE_LIST, STYLE_MESH)
    TYPE_SCATTER = VisType("scatter", Marker.SPHERE_LIST, STYLE_SCATTER)
    TYPE_SPHERE = VisType("sphere", Marker.SPHERE_LIST, STYLE_SPHERE)
    TYPE_CUBE = VisType("cube", Marker.CUBE_LIST, STYLE_CUBE)
    TYPE_ARROW = VisType("arrow", Marker.ARROW, STYLE_ARROW)
    TYPE_ARROW2 = VisType("arrow", Marker.ARROW, STYLE_ARROW2)
    
    def __init__(self, frame_id: str = "odom", topic_name: str = "visualizer_markers"):
        """
        Initialize ROS visualizer
        
        Args:
            frame_id: Frame ID for markers
            topic_name: Topic name for publishing markers
        """
        self.frame_id = frame_id
        self.topic_name = topic_name
        
        # Publisher for markers
        self.marker_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)
        
        # Marker array to store all markers
        self.marker_array = MarkerArray()
        
        # ID management
        self.group_shift = 8  # bit shift
        self.marker_group = 0
        self.marker_subid_list = []  # List of (group_id, sub_id) pairs
        
        # Load parameters
        self._load_params()
        
        rospy.loginfo(f"ROS Visualizer initialized with frame_id: {frame_id}, topic: {topic_name}")
    
    def _load_params(self):
        """Load visualization parameters from ROS parameter server"""
        try:
            # Load frame_id and topic_name
            self.frame_id = rospy.get_param("frame_id", self.frame_id)
            self.topic_name = rospy.get_param("topic_name", self.topic_name)
            
            # Load style parameters
            style_params = [
                ("STYLE_CURVE", self.STYLE_CURVE),
                ("STYLE_MESH", self.STYLE_MESH),
                ("STYLE_FACET", self.STYLE_FACET),
                ("STYLE_SCATTER", self.STYLE_SCATTER),
                ("STYLE_SPHERE", self.STYLE_SPHERE),
                ("STYLE_CUBE", self.STYLE_CUBE),
                ("STYLE_ARROW", self.STYLE_ARROW),
                ("STYLE_ARROW2", self.STYLE_ARROW2)
            ]
            
            for param_name, default_style in style_params:
                if rospy.has_param(param_name):
                    style_values = rospy.get_param(param_name, [default_style.r, default_style.g, default_style.b, default_style.a, default_style.x])
                    if len(style_values) >= 5:
                        default_style.r = style_values[0]
                        default_style.g = style_values[1]
                        default_style.b = style_values[2]
                        default_style.a = style_values[3]
                        default_style.x = style_values[4]
                        if len(style_values) >= 7:
                            default_style.y = style_values[5]
                            default_style.z = style_values[6]
                        
        except Exception as e:
            rospy.logwarn(f"Failed to load visualization parameters: {str(e)}")
    
    def _get_id(self, group_id: int, sub_id: int) -> int:
        """Get marker ID from group and sub IDs"""
        return (group_id << self.group_shift) | sub_id
    
    def _id_update(self, group_id: int = -1) -> int:
        """Update marker ID and return new ID"""
        if group_id > -1:
            self.marker_group = group_id
        
        # Find existing group
        for i, (existing_group, existing_subid) in enumerate(self.marker_subid_list):
            if existing_group == self.marker_group:
                subid = existing_subid + 1
                if subid > (1 << (self.group_shift - 1)):
                    subid = 0
                self.marker_subid_list[i] = (self.marker_group, subid)
                return self._get_id(self.marker_group, subid)
        
        # Create new group
        self.marker_subid_list.append((self.marker_group, 0))
        return self._get_id(self.marker_group, 0)
    
    def set_id_group(self, group_id: int):
        """Set marker group ID"""
        if group_id > -1:
            self.marker_group = group_id
    
    def del_group(self, group_id: int):
        """Delete all markers in a group"""
        self.marker_array.markers = [
            marker for marker in self.marker_array.markers
            if (marker.id >> self.group_shift) != group_id
        ]
        self._publish_markers()
    
    def del_type(self, vis_type: VisType):
        """Delete all markers of a specific type"""
        self.marker_array.markers = [
            marker for marker in self.marker_array.markers
            if marker.ns != vis_type.name_space
        ]
        self._publish_markers()
    
    def del_all(self):
        """Delete all markers"""
        self.marker_array.markers.clear()
        self.marker_subid_list.clear()
        self.marker_group = 0
        self._publish_markers()
    
    def _publish_markers(self):
        """Publish marker array"""
        try:
            self.marker_pub.publish(self.marker_array)
        except Exception as e:
            rospy.logwarn(f"Failed to publish markers: {str(e)}")
    
    def _create_marker(self, vis_type: VisType, marker_id: int) -> Marker:
        """Create a new marker with given type and ID"""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = vis_type.name_space
        marker.id = marker_id
        marker.type = vis_type.marker_type
        marker.action = Marker.ADD
        
        # Set color
        marker.color.r = vis_type.style.r
        marker.color.g = vis_type.style.g
        marker.color.b = vis_type.style.b
        marker.color.a = vis_type.style.a
        
        # Set scale
        marker.scale.x = vis_type.style.x
        marker.scale.y = vis_type.style.y
        marker.scale.z = vis_type.style.z
        
        return marker
    
    def vis_arrow(self, start: np.ndarray, end: np.ndarray, style: VisStyle = None):
        """Visualize an arrow from start to end point"""
        if style is None:
            style = self.TYPE_ARROW.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_ARROW, marker_id)
        
        # Set arrow points
        start_point = Point()
        start_point.x = start[0]
        start_point.y = start[1]
        start_point.z = start[2]
        
        end_point = Point()
        end_point.x = end[0]
        end_point.y = end[1]
        end_point.z = end[2]
        
        marker.points = [start_point, end_point]
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_arrow(self):
        """Delete all arrow markers"""
        self.del_type(self.TYPE_ARROW)
    
    def vis_curve(self, curve: List[np.ndarray], style: VisStyle = None):
        """Visualize a curve as line strip"""
        if style is None:
            style = self.TYPE_CURVE.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_CURVE, marker_id)
        
        # Set curve points
        points = []
        for point in curve:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            points.append(p)
        
        marker.points = points
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_curve(self):
        """Delete all curve markers"""
        self.del_type(self.TYPE_CURVE)
    
    def vis_sphere(self, sphere: np.ndarray, radius: float = None, style: VisStyle = None):
        """Visualize a sphere"""
        if style is None:
            style = self.TYPE_SPHERE.style
        if radius is None:
            radius = style.x
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_SPHERE, marker_id)
        
        # Set sphere position and scale
        marker.pose.position.x = sphere[0]
        marker.pose.position.y = sphere[1]
        marker.pose.position.z = sphere[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def vis_spheres(self, spheres: List[np.ndarray], radius: float = None, style: VisStyle = None):
        """Visualize multiple spheres"""
        if style is None:
            style = self.TYPE_SPHERE.style
        if radius is None:
            radius = style.x
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_SPHERE, marker_id)
        
        # Set sphere list
        points = []
        for sphere in spheres:
            p = Point()
            p.x = sphere[0]
            p.y = sphere[1]
            p.z = sphere[2]
            points.append(p)
        
        marker.points = points
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_sphere(self):
        """Delete all sphere markers"""
        self.del_type(self.TYPE_SPHERE)
    
    def vis_cube(self, cube: np.ndarray, quat: np.ndarray = None, style: VisStyle = None):
        """Visualize a cube"""
        if style is None:
            style = self.TYPE_CUBE.style
        if quat is None:
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_CUBE, marker_id)
        
        # Set cube position and orientation
        marker.pose.position.x = cube[0]
        marker.pose.position.y = cube[1]
        marker.pose.position.z = cube[2]
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        marker.scale.x = style.x
        marker.scale.y = style.y
        marker.scale.z = style.z
        marker.color.r = style.r
        marker.color.g = style.g
        marker.color.b = style.b
        marker.color.a = style.a
        marker.points.append(Point())
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def vis_cubes(self, cubes: List[np.ndarray], quat: np.ndarray = None, style: VisStyle = None):
        """Visualize multiple cubes"""
        if style is None:
            style = self.TYPE_CUBE.style
        if quat is None:
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_CUBE, marker_id)
        
        # Set cube list
        points = []
        for cube in cubes:
            p = Point()
            p.x = cube[0]
            p.y = cube[1]
            p.z = cube[2]
            points.append(p)
        
        marker.points = points
        marker.pose.orientation.w = quat[0]
        marker.pose.orientation.x = quat[1]
        marker.pose.orientation.y = quat[2]
        marker.pose.orientation.z = quat[3]
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_cube(self):
        """Delete all cube markers"""
        self.del_type(self.TYPE_CUBE)
    
    def vis_facet(self, facet: List[np.ndarray], style: VisStyle = None):
        """Visualize a facet (triangle)"""
        if style is None:
            style = self.TYPE_FACET.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_FACET, marker_id)
        
        # Set facet points
        points = []
        for point in facet:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            points.append(p)
        
        marker.points = points
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def vis_facets(self, facets: np.ndarray, style: VisStyle = None):
        """Visualize multiple facets"""
        if style is None:
            style = self.TYPE_FACET.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_FACET, marker_id)
        
        # Set facet points
        points = []
        for i in range(0, facets.shape[0], 3):
            for j in range(3):
                p = Point()
                p.x = facets[i + j, 0]
                p.y = facets[i + j, 1]
                p.z = facets[i + j, 2]
                points.append(p)
        
        marker.points = points
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_facet(self):
        """Delete all facet markers"""
        self.del_type(self.TYPE_FACET)
    
    def vis_mesh(self, mesh: List[np.ndarray], style: VisStyle = None):
        """Visualize a mesh as line list"""
        if style is None:
            style = self.TYPE_MESH.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_MESH, marker_id)
        
        # Set mesh points
        points = []
        for point in mesh:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            points.append(p)
        
        marker.points = points
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def vis_meshes(self, meshes: np.ndarray, style: VisStyle = None):
        """Visualize multiple mesh segments"""
        if style is None:
            style = self.TYPE_MESH.style
        
        marker_id = self._id_update()
        marker = self._create_marker(self.TYPE_MESH, marker_id)
        
        # Set mesh points
        points = []
        for i in range(0, meshes.shape[0], 2):
            for j in range(2):
                p = Point()
                p.x = meshes[i + j, 0]
                p.y = meshes[i + j, 1]
                p.z = meshes[i + j, 2]
                points.append(p)
        
        marker.points = points
        
        # Add to marker array
        self.marker_array.markers.append(marker)
        self._publish_markers()
    
    def del_mesh(self):
        """Delete all mesh markers"""
        self.del_type(self.TYPE_MESH)
    
    def vis_twist(self, pos: np.ndarray, linear: np.ndarray, angular: np.ndarray,
                  linear_style: VisStyle = None, angular_style: VisStyle = None):
        """Visualize twist (linear and angular velocity) as arrows"""
        if linear_style is None:
            linear_style = self.TYPE_ARROW.style
        if angular_style is None:
            angular_style = self.TYPE_ARROW2.style
        
        # Visualize linear velocity
        if np.linalg.norm(linear) > 0.001:
            linear_end = pos + linear * 0.1  # Scale for visualization
            self.vis_arrow(pos, linear_end, linear_style)
        
        # Visualize angular velocity
        if np.linalg.norm(angular) > 0.001:
            angular_end = pos + angular * 0.1  # Scale for visualization
            self.vis_arrow(pos, angular_end, angular_style)
    
    def reset_id(self):
        """Reset marker ID management"""
        self.marker_group = 0
        self.marker_subid_list.clear()


# Convenience functions for backward compatibility
def visArrow(visualizer: ROSVisualizer, start: np.ndarray, end: np.ndarray, style: VisStyle = None):
    """Convenience function for arrow visualization"""
    visualizer.vis_arrow(start, end, style)


def visCurve(visualizer: ROSVisualizer, curve: List[np.ndarray], style: VisStyle = None):
    """Convenience function for curve visualization"""
    visualizer.vis_curve(curve, style)


def visSphere(visualizer: ROSVisualizer, sphere: np.ndarray, radius: float = None, style: VisStyle = None):
    """Convenience function for sphere visualization"""
    visualizer.vis_sphere(sphere, radius, style)


def visCube(visualizer: ROSVisualizer, cube: np.ndarray, quat: np.ndarray = None, style: VisStyle = None):
    """Convenience function for cube visualization"""
    visualizer.vis_cube(cube, quat, style)


def visFacet(visualizer: ROSVisualizer, facet: List[np.ndarray], style: VisStyle = None):
    """Convenience function for facet visualization"""
    visualizer.vis_facet(facet, style)


def visMesh(visualizer: ROSVisualizer, mesh: List[np.ndarray], style: VisStyle = None):
    """Convenience function for mesh visualization"""
    visualizer.vis_mesh(mesh, style)


def visTwist(visualizer: ROSVisualizer, pos: np.ndarray, linear: np.ndarray, angular: np.ndarray,
             linear_style: VisStyle = None, angular_style: VisStyle = None):
    """Convenience function for twist visualization"""
    visualizer.vis_twist(pos, linear, angular, linear_style, angular_style)
