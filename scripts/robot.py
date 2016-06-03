#!/usr/bin/env python
import rospy
from map_utils import Map
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from read_config  import read_config 
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray, Twist
from sklearn.neighbors import NearestNeighbors, KDTree
import numpy as np

class Robot():
   def __init__(self):
      rospy.init_node('robot_node', anonymouse = True)

      self.init_config()
      self.init_particles()

   def init_particles(self):
       

   def init_config(self):
      self.config       = read_config()
      self.anchor_count = config["anchor_count"]
      self.num_nodes    = config["num_nodes"]
      


if __name__ == "__main__":
   ln = Robot()
