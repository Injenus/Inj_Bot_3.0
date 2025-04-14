import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from threading import Lock
import json
import copy

class