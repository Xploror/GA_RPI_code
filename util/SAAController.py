"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Sense and Avoid Algorithm
    @Todo: I don't know
"""

import numpy as np
import math
import util.VectorMath as vmath
import util.SAADataHandling as datahand
import logging
import time

class ObstacleHandle():
    def __init__(self) -> None:
        pass

    def downsampler(self):
        pass
    
    def forget_far_obstacles(self):
        pass
    
    

class ObstacleAvoidance(ObstacleHandle):
    """Obstacle Avoidance Library

    Args:
        ObstacleHandle (Class)
    """
    def __init__(self,max_obs = 10):
        self.vec = vmath.vector()
        self.filter = vmath.Filter()
        self.frame = datahand.DataPreProcessor()    # Required for b2i matrix 
        self.coordinate = datahand.DataPostProcessor() # Required for storing inertial obstacle coordinates
        self.vx = 0
        self.vy = 0
        self.px = 0
        self.py = 0
        self.obstacle_map_ = None
        self.obstacle_map = None
        self.obstacle_inertial = np.array([[1000,1000]])  # Same as self.obstacle_map, but it stores previosuly detected obstacles inertial position wrt origin (takeoff)
        self.mag_obs_inertial = None                      # Magnitude of distance from obstacle to drone for each stored obstacle in self.obstacle_inertial
        self.obstacle_body = None
        self.obstacle_angle = np.array([])                # To store obstacle's angles in front of drone wrt to drone heading (not wrt velocity)
        self.avoid_angle = np.array([])                   # To store obstacle's angles around the drone wrt its heading (for forward maneuver)
        self.brake = 0
        self.guide = 0              # Variable to trigger guidance in GUIDED mode
        self.guiding = 0            # Variable true throughout obstacle avoidance in GUIDED mode
        self.auto = 0               # Variable commanding to switch into AUTO mode, 0 --> stay in current mode  1 --> switch to AUTO mode
        self.ctrl = 0               # Permission for velocity based control of drone given by guided_navigation block (trigger for maneuver block in testutil.py)
        self.stop = 0               # Stopping boolean for guided navigation block
        self.heading = np.zeros(2)  # Heading of drone (Drone front) wrt inertial frame at takeoff position
        self.avoided = 0            # 0 ---> obstacle getting avoided     1 ----> obstacle avoided and drone front is free of obstacles
        self.avoiding = 0           # 1 ---> When not moving forward to avoid obstacle  1 ---->  When moving forward avoiding obstacle
        self.pos_vector = [0,0]

        self.flight_test = 0        # When flight testing it should be one 
  
        self.prev_px = 0
        self.prev_py = 0

        self.engaging_distance = max_obs

        self.mode = "UNKNOWN"

        #Mavlink required to update this stuff
        self.waypoints = np.dot(self.scale(1.01),np.array([[0,0],[0,-188],[150,-188]]).T).T


    def predict_pos_vector(self):
        """
        Predicting the next position of drone. 
        @TODO: This seems to be a very sensitive vector
        Adding an estimator based on position and velocity will be better
        """
        dt = 0.5 #Should be the same as update frequency of previous_position_storer

        #Velocity based direction prediction was extremely sensitive, resulting in skewing of angles
        #Delta position
        self.dvelx = (self.px - self.prev_px)
        self.dvely = self.py - self.prev_py

        #Fusing pos based velocity and raw velocity to get a better estimate 
        self.pos_vector = [self.filter.low_pass_filter(self.dvelx,self.vx*dt),self.filter.low_pass_filter(self.dvely,self.vy*dt)]
    
    
    def scale(self,val):
        """Scaling a 2D vector
        This is intended for waypoint based filtering of obstacle

        Args:
            val (float): Scale

        Returns:
            2x2 matrix: Scaling matrix
        """
        return np.eye(2)*val

    def if_inside_triangle(self,point):
        """Returns a boolean if the point is inisde the waypoint triangle

        Args:
            point (_type_): 2D point
        """
        point += np.array([self.px,self.py])
        a1 = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[1,:],point)
        a2 = self.vec.area_of_triangle(self.waypoints[2,:],self.waypoints[1,:],point)
        a3 = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[2,:],point)
        A = self.vec.area_of_triangle(self.waypoints[0,:],self.waypoints[2,:],self.waypoints[1,:])
        #Returning with some tolerance... this needs to be calibrated. 
        return abs(A - (a1+a2+a3))<=5

    def get_obstacle(self):

        return self.obstacle_map

    def Obstacle_detection(self):
        """Basic detection and updating obstacle_vector from latest obstacle_map
        """
        # Important : This section is to give relavant obstacle_angle needed for right maneuvor only (obstacle avoid)
        if self.avoiding == 0:
            self.obstacle_angle = np.array([])
            if self.obstacle_map is not None:
                obstacle_map_copy = self.obstacle_map
                for i in range(np.size(obstacle_map_copy,axis=0)):
                    obstacle_vector = [obstacle_map_copy[i,0],obstacle_map_copy[i,1]]                          
                    if True:
                            
                        #If obstacle is beyond the specified limits -> don't engage 
                        if(self.vec.mag2d(obstacle_vector)<=0.5 or self.vec.mag2d(obstacle_vector)>=self.engaging_distance):                    
                            self.obstacle_angle = np.append(self.obstacle_angle, 1000)
                        
                        #Compute the angle between the predicted position and obstacle on the field
                        else:
                            self.obstacle_angle = np.append(self.obstacle_angle, round(math.acos(np.dot(self.heading,obstacle_vector)/(self.vec.mag2d(self.heading)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2))
                

        # Important : This section is to give relavant obstacle_angle needed for front maneuvor only (obstacle avoiding)
        else:
            self.avoid_angle = np.array([])
            if self.obstacle_body is not None:
                obstacle_map_copy = self.obstacle_body
                for i in range(np.size(obstacle_map_copy,axis=0)):
                    obstacle_vector = [obstacle_map_copy[i,0],obstacle_map_copy[i,1]]                          
                    if True:
                        
                        #If obstacle is beyond the specified limits -> don't engage 
                        if(self.vec.mag2d(obstacle_vector)<=0.5 or self.vec.mag2d(obstacle_vector)>=self.engaging_distance):                    
                            self.avoid_angle = np.append(self.avoid_angle, 1000)
                        
                        #Compute the angle between the predicted position and obstacle on the field
                        else:
                            self.avoid_angle = np.append(self.avoid_angle, round(math.acos(np.dot(self.heading,obstacle_vector)/(self.vec.mag2d(self.heading)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2))            
                
        obstacle_vector = None  
        obstacle_map_copy = None      

        

    def basic_stop(self):
        """
        Basic Stopping Class
        """
        if self.mode != 'AUTO' or self.obstacle_map is None:
            pass
        #Only move forward if obstacle map is defined
        else:
            # print("Won't ignore this obstacle")
            #print('Hello Im AUTO')
            #Protect the var in for loop
            obstacle_map_copy = self.obstacle_map
            for i in range(np.size(obstacle_map_copy,axis=0)):
                #Compute vector
                obstacle_vector = [obstacle_map_copy[i,0],obstacle_map_copy[i,1]]

                #Scale the triangle using transformation matrix - tomorrow
                #Unless we have a mavlink based waypoint functionality, we should compare this to zero
                #if self.if_inside_triangle(obstacle_vector)==1:                          
                if True:
                    
                    #If drone is not moving or obstacle is beyond the specified limits -> don't engage 
                    if(self.vec.mag2d(self.pos_vector)<=0.02 or self.vec.mag2d(obstacle_vector)<=0.5 or self.vec.mag2d(obstacle_vector)>=self.engaging_distance):                    
                        obstacle_angle = 1000
                    
                    #Compute the angle between the predicted position and obstacle on the field
                    else:
                        obstacle_angle = round(math.acos(np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)

                        #This handling is done for the math domain error. Sometimes the values are outside the 
                        # Cosine domain. This is mainly because to save resource space, I have done a lot of modification
                        # in the raw values.
                        # calc = np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector))
                        # if(abs(calc)<=1):
                        #     obstacle_angle = round(math.acos(calc)*180/math.pi,2)
                        #     #print(obstacle_angle)
                        # else:
                        #     obstacle_angle = math.acos(abs(calc)/calc)
                        #     obstacle_angle = round(math.acos(np.dot(self.pos_vector,obstacle_vector)/(self.vec.mag2d(self.pos_vector)*self.vec.mag2d(obstacle_vector)))*180/math.pi,2)
                            
                    #If angle is in range, engage the brakes                        
                    #@TODO: Range specified by params
                    ##print(f'Obstacle found at : {obstacle_angle}')
                    if abs(obstacle_angle)<10:
                        self.brake = 1
                        print(f"Brake {obstacle_angle} ---  {self.pos_vector} --- {obstacle_vector}")

                else:
                    print("Ignoring obstacle! Good Luck")


        self.Guided_navigation()
        # if self.avoiding == 0:
        #print(f'Guiding : {self.guiding}')
        #print('----------------------')
        print(f'Obstacle Map : {self.obstacle_map}')
        print('----------------------')

        
    def Guided_navigation(self):
        '''Navigating around the obstacle based on map'''
        if self.guiding:
            '''Basic avoidance using obstacle vector using right-front-left maneuver'''
            print('Entered guided navigation mode')
            self.brake = 0
            self.Obstacle_detection() 
            if self.avoiding == 0:
                #print(f'                          {[i for i in self.obstacle_angle if i < 10]}')
                if np.any(abs(self.obstacle_angle) < 10):   # 10 deg for avoiding obstacles
                    self.avoided = 0         #  This will
                    self.ctrl = 1            #  make the drone 
                    self.stop = 0            #  move right unless no obstacle in-front
                else:
                    self.stop = 1            
                    #time.sleep(1)            # This will stop drone for 5 seconds when no obstacle in-front
                    self.avoiding = 1
                    #time.sleep(1)
                    print('NO obstacle between +-10 deg')
            else:
                if np.any(abs(self.avoid_angle) < 80):   # Usually delay in data so 80 deg else targetting 100 deg clearance
                    if np.any(abs(self.avoid_angle) < 10):
                        self.avoiding = 0  #   If any obstacle is still under 10 degrees then switch to right maneuver
                    else:
                        self.avoided = 1       #  This will call
                        self.stop = 0          #  Maneuver forward command
                else:
                    self.auto = 1          #   Obstacle avoided so change back to AUTO mode 
                    self.guiding = 0       #   and disable all methods triggered by guiding

        self.obstacle_angle = np.array([])
        self.avoid_angle = np.array([])            
                
                
            
            # self.guiding = 0
            # self.auto = 1
          
            
