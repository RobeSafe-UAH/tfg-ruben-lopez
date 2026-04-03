import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from norfair import Detection, Tracker

from .libraries.converts_library import yaw2quaternion