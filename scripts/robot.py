#!/usr/bin/env python
import rospy
import helper_functions
import random
import math
from map_utils import Map
from nav_msgs.msg import OccupancyGrid
from read_config  import read_config 
from geometry_msgs.msg import Pose, PoseArray
#from cse_190_assi_2.msg import 
#from cse_190_assi_2.srv import 

class Robot():
   def __init__(self):
      
      rospy.init_node('robot_node', anonymous = True)

      self.init_map_sub()
      self.init_pubs()
      rospy.sleep(2) #make sure we create the Map object in init_subs
      self.init_config() 
      self.init_particles()
      self.publish_particles()


      rospy.spin()

   def init_config(self):
      self.config             = read_config()
      self.num_particles      = self.config["num_particles"]
      self.map_width          = self.myMap.width
      self.map_height         = self.myMap.height

   def init_map_sub(self):
      self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.handle_map_reply)

   def init_pubs(self):
      self.particle_cloud_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size = 10)

   def init_particles(self):
      self.particle_ls = []
      for i in range (0, self.num_particles):
         r_x     = random.uniform(0, self.map_width)
         r_y     = random.uniform(0, self.map_height)
         r_theta = random.uniform(0, 2*math.pi)
         p = Particle(r_x, r_y, r_theta, 1/self.num_particles)
         self.particle_ls.append(p)


   def handle_map_reply(self, grid):
      self.myMap = Map(grid) 

   def publish_particles(self):
      pose_arr = PoseArray()
      pose_arr.header.stamp = rospy.Time.now()
      pose_arr.header.frame_id = 'map'
      pose_arr.poses = []
      for p in self.particle_ls:
         pose_arr.poses.append(p.pose) 

      self.particle_cloud_pub.publish(pose_arr) 

class Particle():
   def __init__(self, x, y, theta, weight):
      self.x = x
      self.y = y
      self.theta = theta
      self.weight = weight
      self.pose = helper_functions.get_pose(x, y, theta)

if __name__ == "__main__":
   ln = Robot()
