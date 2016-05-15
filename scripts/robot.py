#!/usr/bin/env python
import rospy
import helper_functions
import random
import math
import copy
import itertools
from map_utils import Map
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from read_config  import read_config 
from geometry_msgs.msg import Pose, PoseArray, Twist
from sklearn.neighbors import NearestNeighbors, KDTree
import numpy as np
#from cse_190_assi_2.msg import 
#from cse_190_assi_2.srv import 

class Robot():
   def __init__(self):
      
      rospy.init_node('robot_node', anonymous = True)
      self.map_inited = 0;
      self.move_num = 0
      self.first_move = True

      self.init_map_sub()
      self.init_pubs()
      rospy.sleep(3) #make sure we create the Map object in init_subs
      self.init_config() 
      self.init_particles()
      self.publish_particles()

      self.init_likelihood_map()

      self.start_moves()

      rospy.spin()

   def init_config(self):
      self.config             = read_config()
      self.num_particles      = self.config["num_particles"]
      self.map_width          = self.myMap.width
      self.map_height         = self.myMap.height
      self.laser_sig_hit      = self.config["laser_sigma_hit"]
      self.move_list          = self.config["move_list"]
      self.fm_sigma_x         = self.config["first_move_sigma_x"]
      self.fm_sigma_y         = self.config["first_move_sigma_y"]
      self.fm_sigma_ang       = self.config["first_move_sigma_angle"]
      self.total_moves        = len(self.move_list)
      self.laser_z_hit        = self.config["laser_z_hit"]
      self.laser_z_rand       = self.config["laser_z_rand"]
      self.resamp_sig_x       = self.config["resample_sigma_x"]
      self.resamp_sig_y       = self.config["resample_sigma_y"]
      self.resamp_sig_a       = self.config["resample_sigma_angle"]
      self.turned             = False

   def init_map_sub(self):
      self.map_sub  = rospy.Subscriber('/map', OccupancyGrid, self.handle_map_reply)
      self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.handle_scan)


   def init_pubs(self):
      self.particle_cloud_pub = rospy.Publisher('/particlecloud', PoseArray, 
                                                queue_size = 10, latch = True)
      self.likelihood_pub     = rospy.Publisher('/likelihood_field', OccupancyGrid, 
                                                queue_size = 10, latch = True)

   def handle_map_reply(self, grid):
      if self.map_inited     == 0:
         self.myMap          = Map(grid) 
         self.likelihood_map = Map(grid)
         self.map_inited     = 1

   def handle_scan(self, scan):
      self.scan    = scan
      self.num_sensors = (scan.angle_max - scan.angle_min) / scan.angle_increment
      self.scan_ls = scan.ranges
      self.scan_ang_min = scan.angle_min

   def init_likelihood_map(self):
      occupied_points     = self.get_occupied_points()
      near_ls             = self.get_nearest_neighbors(occupied_points)
      it                  = 0
      for x in range (0, self.map_width):
         for y in range (0, self.map_height):
            val = self.gaussian_likelihood(near_ls[it][0], 0, self.laser_sig_hit)
            self.likelihood_map.set_cell(x, y, val)
            it+= 1
      self.publish_likelihood_map()

   def get_occupied_points(self):
      occupied_points = []
      for x in range ( 0, self.map_width):
         for y in range ( 0, self.map_height):
            if self.likelihood_map.get_cell(x, y) == 1:
               occupied_points.append( [x , y])
      return occupied_points

   def publish_likelihood_map(self):
      message = self.likelihood_map.to_message()
      self.likelihood_pub.publish(message)
      

   def gaussian_likelihood(self, x1, x2, sigma):
      #alpha = 1 / ( sigma * math.sqrt( 2 * math.pi ))
      ex    = math.exp(-(x1 - x2)**2 / 2 * sigma**2 )
      res   =  ex
      return res

   def get_nearest_neighbors(self, occu_pt):
      allpts = []
      for x in range (0, self.map_width):
         for y in range (0, self.map_height):
            allpts.append([x,y])
      allpts = np.array(allpts)

      x    = np.array(occu_pt)
      kdt  = KDTree(x, leaf_size=30, metric='euclidean') 
      dist, ind = kdt.query(allpts, k=1, return_distance=True)
      return dist


   def publish_particles(self):
      pose_arr = PoseArray()
      pose_arr.header.stamp = rospy.Time.now()
      pose_arr.header.frame_id = 'map'
      pose_arr.poses = []
      for p in self.particle_ls:
         pose_arr.poses.append(p.pose) 

      self.particle_cloud_pub.publish(pose_arr) 

   def start_moves(self):
      move_count = 0
      while move_count < self.total_moves: 
         self.move_robot()
         self.publish_particles()
         self.turned = False
         move_count += 1

   def move_robot(self):
      angle, dist, steps = self.get_move() 
      #turn robot
      helper_functions.move_function(angle, 0)
      p_theta_mov = math.radians(angle)

      for p in self.particle_ls:
         p.set_theta( p.theta+p_theta_mov)
         p.update_pose()

      #move robot
      for n in range (0, steps):
         helper_functions.move_function(0, dist)
         print "moved robot"

         #move particles?

         self.move_particles()
         self.publish_particles()
         print "moved particles"

         self.reweigh_all_particles()
         self.publish_particles()
         print "reweighed"

         self.resample_particles()
         self.publish_particles()
         print "resampled"

         self.publish_particles()
         print "published"


   def get_move(self):
      self.current_move = self.move_list[self.move_num]
      self.move_num += 1
      return self.current_move

   def resample_particles(self):
      pick_ls = [] 
      for p in self.particle_ls:
         p_amount = [copy.deepcopy(p)] * int(round(self.num_particles * p.weight))
         pick_ls.append(p_amount)
      picked_ls_flat = list(itertools.chain(*pick_ls))

      new_ls = []
      for i in range (0, self.num_particles):
         picked       = copy.deepcopy(random.choice(picked_ls_flat))
         x     = random.gauss(picked.x, self.resamp_sig_x)
         y     = random.gauss(picked.y, self.resamp_sig_y)
         #theta = np.random.normal(picked.theta, self.resamp_sig_a, 1)
         theta = random.gauss(picked.theta, self.resamp_sig_a)

         resampled_p  = Particle(x, y, theta, picked.weight)

         #picked.update_pose()
         new_ls.append(resampled_p)

      self.particle_ls = copy.deepcopy(new_ls)

   def reweigh_all_particles(self):
      Lp = 0 
      self.laser_ind = 0
      for p in self.particle_ls:
         p_tot = 0
         num_shit_sensors = 0

         for s in self.scan_ls:
            laser_angle_local = self.scan_ang_min+ self.scan.angle_increment*self.laser_ind
            x = s*math.cos(laser_angle_local+p.theta) + p.x
            y = s*math.sin(laser_angle_local+p.theta) + p.y
            Lp = self.likelihood_map.get_cell(x, y)
            if math.isnan(Lp):
               num_shit_sensors += 1
               Lp = 0
            Pz    = self.laser_z_hit*Lp + self.laser_z_rand
            p_tot += Pz
            self.laser_ind += 1
         if p.x < 0 or p.y < 0 or p.x >= self.map_width or p.y >= self.map_height:
            p.weight = 0
         elif self.likelihood_map.get_cell(p.x, p.y) == 1:
            p.weight = 0
         elif num_shit_sensors / self.num_sensors > 0.6:
            p.weight = 0
         else:
            p.weight = p.weight *(1/(1+math.exp(-1*p_tot)))
      self.normalize_particles()
      
   def normalize_particles(self):
      su = 0
      for p in self.particle_ls:
         su += p.weight
      for p in self.particle_ls:
         p.weight = p.weight / su

   def move_particles(self):
      #TODO: list comprehension moved_list = [
      ang, dist, steps      = self.current_move
      angle_rad = math.radians(ang)
      
      #move every particle
      for p in self.particle_ls:
         x_bar = dist*math.cos(p.theta)
         y_bar = dist*math.sin(p.theta)

         if self.first_move:
            print "first move"
            #add noise
            angl, x, y = self.handle_first_particle_move(x_bar, y_bar,p.theta)
           # p.theta = p.theta+angl
            p.x = p.x + x_bar
            p.y = p.y + y_bar
            p.update_pose()

         else:
            #p.move_function(math.degrees(angl), 0)
            #p.move_function(0, dist)
            #if self.turned == False:
           #    print "before: " , p.theta
           #    p.set_theta( p.theta+angle_rad)
           #    p.update_pose()
           #    print "after: " , p.theta
            p.x = p.x + x_bar
            p.y = p.y + y_bar
            p.update_pose()

      self.first_move = False
     # self.turned = True

   def handle_first_particle_move(self, x_bar, y_bar, angle):
      mu      = 0
      sig_x                   = self.fm_sigma_x
      sig_y                   = self.fm_sigma_y
      sig_ang                 = self.fm_sigma_ang
      theta = random.gauss(angle, sig_ang)
      x     = random.gauss(x_bar,     sig_x)
      y     = random.gauss(y_bar,     sig_y)
      return theta, x, y
         
         
   def distance_moved_in_xy(self, d, n, theta):
      return x, y

   def gaussian(self, x, x2, sigma):
      alpha = 1 / ( sigma * math.sqrt( 2 * math.pi ))
      ex    = math.exp(-(x - x2)**2 / 2 * sigma**2 )
      res   =  alpha * ex
      return res

   def init_particles(self):
      self.particle_ls = []
      for i in range (0, self.num_particles):
         r_x     = random.uniform(0, self.map_width)
         r_y     = random.uniform(0, self.map_height)
         r_theta = random.uniform(0, 2*math.pi)
         p       = Particle(r_x, r_y, r_theta, 1./self.num_particles)

         self.particle_ls.append(p)


class Particle():
   def __init__(self, x, y, theta, weight):
      self.x = x
      self.y = y
      self.theta = theta
      self.weight = weight
      self.pose = helper_functions.get_pose(x, y, theta)

   def update_pose(self):
      self.pose = helper_functions.get_pose(self.x, self.y, self.theta)
   def set_theta(self, theta):
      self.theta = theta

   def move_function (self, angle, dist):
      #print "stuck here?"
      angle_in_rad = float(angle*math.pi/180.0)
      if angle > 0:
         increment = 10	
      else:
         increment = -10

      increment_in_rad = float(increment*math.pi/180.0)
      while angle_in_rad != 0:
         twist = Twist()
         if abs(angle_in_rad) > abs(increment_in_rad) :
            twist.angular.z = increment_in_rad
            angle_in_rad -= increment_in_rad
         else:
            twist.angular.z = angle_in_rad
            angle_in_rad = 0
         twist = Twist()

      twist = Twist()
      twist.linear.x = dist
      twist = Twist()







if __name__ == "__main__":
   ln = Robot()
