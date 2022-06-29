"""
@author: Dhruv Parikh
@organisation: General Aeronautics Pvt. Ltd.
"""

"""
@Description:
    Vector math library providing helper function for estimator design
"""

import math
import numpy as np


class AngularTransformation():
    
    def __init__(self):
        pass

    def calc_trig_values(self,roll,pitch,yaw):
        """
        Description: Calculates sin and cosine to avoid repetitive computation
        **Important**
        Run always before calling other functions in this class to update angles
        @param: Roll, Pitch and Yaw in radians
        @output: None
        """
        self.cosr = math.cos(roll)
        self.sinr = math.sin(roll)
        self.cosp = math.cos(pitch)
        self.sinp = math.sin(pitch)
        self.cosy = math.cos(yaw)
        self.siny = math.sin(yaw)

    def rotx(self):
        """
        @param: None
        @output:3x3 matrix rotated in X axis
        """
        X = np.matrix([[1,0,0],[0,self.cosr,-self.sinr],[0,self.sinr,self.cosr]])
        return X
    
    def roty(self):
        """
        @param: None
        @output:3x3 matrix rotated in Y axis
        """

        Y = np.matrix([[self.cosp,0,self.sinp],[0,1,0],[-self.sinp,0,self.cosp]])
        return Y
    
    def rotz(self):
        """
        @param: None
        @output:3x3 matrix rotated in Z axis
        """

        Z = np.matrix([[self.cosy,-self.siny,0],[self.siny,self.cosy,0],[0,0,1]])
        return Z

    def euler_zyx(self):
        """
        @param: None
        @output:3x3 matrix rotated in ZYX axis  inertial to body axis

        This is 4 ms slower than current one
        #return np.dot(np.dot(self.rotz(),self.roty()),self.rotx())
        """
        X = np.matrix([[1,0,0],[0,self.cosr,-self.sinr],[0,self.sinr,self.cosr]])
        Y = np.matrix([[self.cosp,0,self.sinp],[0,1,0],[-self.sinp,0,self.cosp]])
        Z = np.matrix([[self.cosy,-self.siny,0],[self.siny,self.cosy,0],[0,0,1]])
        return np.dot(np.dot(Z,Y),X)
        

    def quat_rot_matrix(self,Q):
        """
        @param: Q quaternion (4 lengthed list)
        @description:

        6.5x Faster than Rotation matrix
        Saves 15 ms per lidar scan (on my pc atleast)
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
                
        # 3x3 rotation matrix
        rot_matrix = np.array([[2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                            [2 * (q1 * q2 + q0 * q3), 2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1)],
                            [2 * (q1 * q3 - q0 * q2),  2 * (q2 * q3 + q0 * q1), 2 * (q0 * q0 + q3 * q3) - 1]])
        
        return rot_matrix


class LateralTransformation():
    """
    GPS coords to meters and vice versa
    """
    def __init__(self) -> None:
        pass
    def distance_between_two_points_2d(self):
        pass
    def distance_between_two_points_3d(self):
        pass        
    
    def meters_2_latlon(self,dx,dy,lat,lng):
        """
        @param: delta x, delta y and origin latitutde longitude in degrees
        @output:returns latitude and longitude of the new coordinates
        @description: 
        Functions returns the lat lon of the new coordinates
        Reference: APM Copter 4.1.1
        """
        dlat = dx/0.01113188453
        dlng = dy/(0.01113188453*self.longitude_scale(lat+dlat/2))
        lat += dlat
        lng = dlng+lng
        return lat, lng


    def longitude_scale(self,lat):
        """
        @param: latitude
        @description: 
        Longitude scale for the poles
        Reference: APM Copter 4.1.1
        """
        return max(math.cos(lat*math.pi*1e-7/180),0.01)

    def gps_2_meters(self,lat1,lon1,lat2,lon2):
        """
        @param: delta x, delta y and origin latitutde longitude in degrees
        @output:returns latitude and longitude of the new coordinates
        @description: 
        Functions returns the lat lon of the new coordinates
        Reference: APM Copter 4.1.1
        """
        scalar = 0.01113188453
        x = (lat2 - lat1)* scalar

        if(lon1*lon2>=0):
            y = (lon2 - lon1)*scalar*self.longitude_scale((lat1+lat2)/2)
            return x,y
        
        dlon = lon2 - lon1

        if dlon>1800000000:
            dlon -= 3600000000
        elif dlon< -1800000000:
            dlon += 3600000000

        y = dlon*scalar*self.ongitude_scale((lat1+lat2)/2)
        return x,y



class Filter():
    def __init__(self) -> None:
        pass
    def low_pass_filter(self,previous_val,current_val):
        """A simple 20% cutoff low pass filter
        Current I am using this as a fusion filter
        Args:
            previous_val (float): Memory
            current_val (float): Current
        
        Return:

        """
        return 0.8*previous_val + 0.2*current_val

    def three_window_fast_median_filter(self):
        pass


class vector():

    def __init__(self):
        print("Vector Class initialized")

    def mag2d(self,v):
        return math.sqrt(float(v[0]**2 + v[1]**2))

    def return_sign(self,var):
        if var!=0:
            return abs(var)/var
        else:
            return 1

    def area_of_triangle(self,p1, p2, p3):
        """
        Area of a triangle
        """
        x1,y1 = float(p1[0]),float(p1[1])
        x2,y2 = float(p2[0]),float(p2[1])
        x3,y3 = float(p3[0]),float(p3[1])
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                + x3 * (y1 - y2)) / 2.0)