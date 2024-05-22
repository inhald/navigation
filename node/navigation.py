#!/usr/bin/env python3
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan, Imu
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# camera imports
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

from tf.transformations import euler_from_quaternion


import geometry_msgs
import struct


class GapBarrier:
    def __init__(self):

        #Topics & Subs, Pubs
        # Read paramters from params.yaml
        depth_image_topic =rospy.get_param('~depth_image_topic')
        depth_info_topic =rospy.get_param('~depth_info_topic')
        cv_ranges_topic =rospy.get_param('~cv_ranges_topic')
        depth_index_topic =rospy.get_param('~depth_index_topic')
        depth_points_topic =rospy.get_param('~depth_points_topic')
        lidarscan_topic =rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        odom_topic=rospy.get_param('~odom_topic')
        mux_topic = rospy.get_param('~mux_topic')
        imu_topic = rospy.get_param('~imu_topic')

        self.nav_mux_idx= rospy.get_param('~nav_mux_idx')

        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.CenterOffset=rospy.get_param('~CenterOffset')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.tau=rospy.get_param('~tau')

        self.min_cv_range=rospy.get_param('~min_cv_range')
        self.max_cv_range=rospy.get_param('~max_cv_range')
        self.cv_distance_to_lidar=rospy.get_param('~cv_distance_to_lidar')
        self.num_cv_sample_rows=rospy.get_param('~num_cv_sample_rows')
        self.num_cv_sample_cols=rospy.get_param('~num_cv_sample_cols')

        self.cv_ground_angle=rospy.get_param('~cv_ground_angle')
        self.cv_lidar_range_max_diff=rospy.get_param('~cv_lidar_range_max_diff')
        self.camera_height=rospy.get_param('~camera_height')
        self.cv_real_to_theo_ground_range_ratio=rospy.get_param('~cv_real_to_theo_ground_range_ratio')
        self.cv_real_to_theo_ground_range_ratio_near_horizon=rospy.get_param('~cv_real_to_theo_ground_range_ratio_near_horizon')
        self.cv_ground_range_decay_row=rospy.get_param('~cv_ground_range_decay_row')
        self.cv_pitch_angle_hardcoded=rospy.get_param('~cv_pitch_angle_hardcoded')

        self.angle_bl=rospy.get_param('~angle_bl')
        self.angle_al=rospy.get_param('~angle_al')
        self.angle_br=rospy.get_param('~angle_br')
        self.angle_ar=rospy.get_param('~angle_ar')
        self.n_pts_l=rospy.get_param('~n_pts_l')
        self.n_pts_r=rospy.get_param('~n_pts_r')
        self.vehicle_velocity=rospy.get_param('~vehicle_velocity')
        self.turn_velocity=rospy.get_param('~turn_velocity')
        self.turn_angle1 = rospy.get_param('~turn_angle1')
        self.turn_angle2 = rospy.get_param('~turn_angle2')
        self.stop_time1 = rospy.get_param('~stop_time1')
        self.stop_time2 = rospy.get_param('~stop_time2')
        self.scan_beams=rospy.get_param('~scan_beams')
        self.safe_distance=rospy.get_param('~safe_distance')
        self.right_beam_angle=rospy.get_param('~right_beam_angle')
        self.left_beam_angle=rospy.get_param('~left_beam_angle')
        self.heading_beam_angle=rospy.get_param('~heading_beam_angle')
        self.stop_distance=rospy.get_param('~stop_distance')
        self.stop_distance_decay=rospy.get_param('~stop_distance_decay')
        self.velocity_zero=rospy.get_param('~velocity_zero')
        self.optim_mode=rospy.get_param('~optim_mode')
        self.use_camera=rospy.get_param('~use_camera')
        self.max_lidar_range_opt = rospy.get_param('~max_lidar_range_opt')

        
        self.counter = 0
        self.vel = 0
        self.ls_ang_inc=2*math.pi/self.scan_beams
        self.nav_active = 0

        # Camera setup
        self.cvbridge = CvBridge()
        self.intrinsics = None
        self.cv_image_data = None
        self.cv_ranges = None
        self.cv_beam_indices = None

        # Define field of view (FOV) to search for obstacles 

        self.ls_str=int(round(self.scan_beams*self.right_beam_angle/(2*math.pi)))
        self.ls_end=int(round(self.scan_beams*self.left_beam_angle/(2*math.pi)))
        self.ls_len_mod=self.ls_end-self.ls_str+1
        self.ls_fov=self.ls_len_mod*self.ls_ang_inc
        self.angle_cen=self.ls_fov/2
        self.ls_len_mod2=0
        self.ls_data=[]

        self.drive_state="normal"
        self.stopped_time =0.0
        self.yaw0 =0.0 
        self.dtheta = 0.0 
        self.yaw = 0.0
        self.imu_roll =0
        self.imu_pitch =0
        self.imu_yaw =0
        t = rospy.Time.from_sec(time.time())
        self.current_time = t.to_sec()
        self.prev_time = self.current_time
        self.time_ref = 0.0
        self.wl0 = np.array([0.0, -1.0])
        self.wr0 = np.array([0.0, 1.0])

        # Subscriptions and Publications 

        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)
        rospy.Subscriber(mux_topic, Int32MultiArray,self.mux_callback, queue_size=1)
        rospy.Subscriber(imu_topic, Imu,self.imu_callback, queue_size=1)

        self.marker_pub = rospy.Publisher("wall_markers", Marker, queue_size = 2)
        self.marker = Marker()
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)


        if self.use_camera == 1:
            rospy.Subscriber(depth_image_topic, Image, self.imageDepth_callback,queue_size=1)
            rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfo_callback,queue_size=1)
            rospy.Subscriber(depth_image_topic.replace('depth', 'confidence'), Image, self.confidenceCallback)
        
            ############ For camera debug purpose ############
            # camera data corresponding lidar beam indices
            self.cv_ranges_msg = LaserScan()
            self.cv_ranges_msg.header.frame_id = 'laser'
            self.cv_ranges_msg.angle_increment = self.ls_ang_inc
            self.cv_ranges_msg.time_increment = 0
            self.cv_ranges_msg.range_min = 0
            self.cv_ranges_msg.range_max = self.max_lidar_range
            self.cv_ranges_msg.angle_min = 0  #self.cv_str*self.ls_ang_inc
            self.cv_ranges_msg.angle_max = 2*math.pi #self.cv_end*self.ls_ang_inc

            # Publications  
            self.cv_ranges_pub = rospy.Publisher(cv_ranges_topic, LaserScan, queue_size=1)
        

        
    def mux_callback(self, mux_data):

        self.nav_active = mux_data.data[self.nav_mux_idx]     


    def augment_camera(self, lidar_ranges):
        # cv_image is 2D numpy array of depth values
        # Realsense D435: cv_image.shape = (480, 848), video quality drops after some time and becomes (480,640), or it could be caused by USB2 wire instead of USB3
        cv_image = self.cvbridge.imgmsg_to_cv2(self.cv_image_data, self.cv_image_data.encoding)
        # project camera data to 2D lidar coordinate
        # col: to the right, sometimes called x
        # row: downwards, sometimes called y
        assert ((self.cv_rows, self.cv_cols) == cv_image.shape)
        # @TODO: cv_ranges = self.last_cv_image*expand_factor
        #cv_ranges = np.full(len(self.cv_scan_cols), np.inf)

        for col in self.cv_sample_cols_raw:

            for row in self.cv_sample_rows_raw:
                
                depth = cv_image[row, col]/1000
                
                if depth > self.max_cv_range or depth < self.min_cv_range:
                    continue 
                
                cv_point = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [col, row], depth)
                (cv_coordx, cv_coordy, cv_coordz) = cv_point

                self.imu_pitch =0
                self.imu_roll =0 
                # cv_coordx_s = cv_coordx*math.cos(self.imu_pitch)+cv_coordy*math.sin(self.imu_pitch)*math.cos(self.imu_roll)+cv_coordz*math.sin(self.imu_pitch)*math.sin(self.imu_roll)
                cv_coordy_s = -cv_coordx*math.sin(self.imu_pitch)+cv_coordy*math.cos(self.imu_pitch)*math.cos(self.imu_roll) + cv_coordz *math.cos(self.imu_pitch)*math.sin(self.imu_roll)
                # cv_coordz_s =  - cv_coordy*math.sin(self.imu_roll) + cv_coordz*math.cos(self.imu_roll)
                
                if cv_coordy_s > 0.5*self.camera_height or cv_coordy_s < -2.5*self.camera_height:
                    continue

                lidar_coordx = -(cv_coordz+self.cv_distance_to_lidar)
                lidar_coordy = cv_coordx
                cv_range_temp = math.sqrt(lidar_coordx**2+lidar_coordy**2)


                beam_index = math.floor(self.scan_beams*math.atan2(lidar_coordy, lidar_coordx)/(2*math.pi))
                lidar_range = lidar_ranges[beam_index]
                lidar_ranges[beam_index] = min(lidar_range, cv_range_temp)


        # Camera debugging
        self.cv_ranges_msg.header.stamp = rospy.Time.now()
        self.cv_ranges_msg.ranges = lidar_ranges
        self.cv_ranges_pub.publish(self.cv_ranges_msg)

        return


    def imageDepth_callback(self, data):
        try:
            if self.intrinsics:
                self.cv_image_data = data

            else:
                self.cv_image_data = None
        except Exception as e:
            print(e)
            return

    # Realsense D435 has no confidence data
    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
            print("confidenceCallback")
        except Exception as e:
            print(e)
            return

    def imageDepthInfo_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
            print('Intrinsics: %s' % self.intrinsics)


            # For camera debugging

            self.cv_rows = self.intrinsics.height
            self.cv_cols = self.intrinsics.width


            self.cv_sample_rows_raw = np.linspace(100, self.cv_rows-1, num=self.num_cv_sample_rows, dtype=int)
            self.cv_sample_cols_raw = np.linspace(0, self.cv_cols-1, num=self.num_cv_sample_cols, dtype=int)


        except Exception as e:
            print(e)
            return


    # Pre-process LiDAR data    
    def preprocess_lidar(self, ranges):
        
        data=[]
        data2=[]     

        for i in range(self.ls_len_mod):
            if ranges[self.ls_str+i]<=self.safe_distance:
               data.append ([0,i*self.ls_ang_inc-self.angle_cen])
            elif ranges[self.ls_str+i]<=self.max_lidar_range:
                data.append ([ranges[self.ls_str+i],i*self.ls_ang_inc-self.angle_cen])
            else: 
                data.append ([self.max_lidar_range,i*self.ls_ang_inc-self.angle_cen])
        
        k1 = 100
        k2 = 40

        for i in range(k1):
            
            s_range = 0

            for j in range(k2):
                index1 = int(i*len(ranges)/k1+j)
                if index1 >= len(ranges):
                    index1 = index1-len(ranges)
                    
                index2 = int(i*len(ranges)/k1-j)
                if index2 < 0:
                    index2= index2 + len(ranges)    
                
                s_range = s_range + min(ranges[index1],self.max_lidar_range)+ min(ranges[index2],self.max_lidar_range)

            data2.append(s_range)
             
             
        return np.array(data), np.array(data2)



    # Return the start and end indices of the maximum gap in free_space_ranges
    def find_max_gap(self, proc_ranges):
        
        j=0
        str_indx=0;end_indx=0
        str_indx2=0;end_indx2=0
        
        range_sum = 0
        range_sum_new = 0

        for i in range (self.ls_len_mod):

            if proc_ranges[i,0]!=0:
                if j==0:
                    str_indx=i
                    range_sum_new = 0
                    j=1
                range_sum_new = range_sum_new + proc_ranges[i,0]
                end_indx=i
                
            if  j==1 and (proc_ranges[i,0]==0 or i==self.ls_len_mod-1):
               
                j=0

                if  range_sum_new > range_sum: 
                        end_indx2=end_indx
                        str_indx2=str_indx
                        range_sum = range_sum_new

        return str_indx2, end_indx2

    
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):

        range_sum =0
        best_heading =0

        for i in range (start_i, end_i+1):
            range_sum = range_sum+proc_ranges[i,0]
            best_heading = best_heading+proc_ranges[i,0]*proc_ranges[i,1]

        if range_sum != 0:
             best_heading = best_heading/range_sum
        
        return best_heading


 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):

        
        if self.optim_mode == 0:
            
            Pr = np.array([[1.0,0],[0,1.0]])
            Pl = np.array([[1.0,0],[0,1.0]])

            n_obs_l = len(left_obstacles)
            n_obs_r = len(right_obstacles)

            bl = np.full(n_obs_l, 1.0, dtype=np.float64)
            br = np.full(n_obs_r, 1.0, dtype=np.float64)
                   
            Cl=-(left_obstacles.T)
            Cr= -(right_obstacles.T)
            al = (1-alpha)*wl0 
            ar = (1-alpha)*wr0 

            wl = solve_qp(Pl.astype(np.float), al.astype(np.float), Cl.astype(np.float),  bl.astype(np.float), 0)[0]
            wr = solve_qp(Pr.astype(np.float), ar.astype(np.float), Cr.astype(np.float),  br.astype(np.float), 0)[0]

        else: 

            n_obs_l = len(left_obstacles)
            n_obs_r = len(right_obstacles)

            P = np.array([[1.0,0,0],[0,1.0,0],[0,0,0.0001]])

            bl = np.full(n_obs_l, 1.0, dtype=np.float64)
            br = np.full(n_obs_r, 1.0, dtype=np.float64)        
            b = np.concatenate((br,bl,np.array([-0.90, -0.90])))

            Cl=-(left_obstacles.T)
            Cr= -(right_obstacles.T)
            C1 = np.vstack((-Cr,br))
            C2 = np.vstack((Cl,- bl))
            C =np.hstack((C1,C2))
            C =np.hstack((C,np.array([[0,0],[0,0],[1.0,-1.0]])))
        
            a = np.zeros(3)

            ws  = solve_qp(P.astype(np.float), a.astype(np.float), C.astype(np.float),  b.astype(np.float), 0)[0]
        
            wr = np.array([ws[0]/(ws[2]-1),ws[1]/(ws[2]-1)])
            wl = np.array([ws[0]/(ws[2]+1),ws[1]/(ws[2]+1)])
        
        return  wl, wr 


    def imu_callback(self, data):
        
        Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        
        (self.imu_roll, self.imu_pitch, self.imu_yaw) = euler_from_quaternion(Imu_msg)

        #print (self.imu_roll, self.imu_pitch)

        return



    def lidar_callback(self, data):    

        self.ls_ang_inc = data.angle_increment
        self.scan_beams = int(2*math.pi/data.angle_increment)
        self.ls_str=int(round(self.scan_beams*self.right_beam_angle/(2*math.pi)))
        self.ls_end=int(round(self.scan_beams*self.left_beam_angle/(2*math.pi)))

        fused_ranges = list(data.ranges)

        if self.use_camera == 1:

            if self.cv_image_data is not None:
                self.augment_camera(fused_ranges)
    
        if self.nav_active == 0:
            
            self.drive_state == "normal"
            return
  

        t = rospy.Time.from_sec(time.time())
        self.current_time = t.to_sec()
        dt = self.current_time - self.prev_time
        self.prev_time = self.current_time
        sec_len= int(self.heading_beam_angle/data.angle_increment)
        
        # proc_ranges takes backward as 0 angle, and CCW as positive direction
        # proc_ranges ranges from left to right angle
        # aug_ranges combines lidar_ranges and cv_ranges

        proc_ranges, mod_ranges = self.preprocess_lidar(fused_ranges)     


        if self.drive_state == "normal":

            str_indx,end_indx=self.find_max_gap(proc_ranges)           
            heading_angle =self.find_best_point(str_indx, end_indx, proc_ranges)
        

            index_l=int(round((self.angle_bl-self.angle_al)/(data.angle_increment*self.n_pts_l)))
            index_r=int(round((self.angle_ar-self.angle_br)/(data.angle_increment*self.n_pts_r)))

            mod_angle_al = self.angle_al + heading_angle

            if mod_angle_al > 2*math.pi:
                mod_angle_al = mod_angle_al - 2*math.pi
            elif mod_angle_al < 0:
                mod_angle_al = mod_angle_al + 2*math.pi
        
            mod_angle_br = self.angle_br + heading_angle

            if mod_angle_br > 2*math.pi:
                mod_angle_br = mod_angle_br - 2*math.pi
            
            elif mod_angle_br < 0:
                mod_angle_br = mod_angle_br + 2*math.pi
 

            start_indx_l=int(round(mod_angle_al/data.angle_increment))
            start_indx_r=int(round(mod_angle_br/data.angle_increment))

            obstacle_points_l=np.array([[0, self.max_lidar_range_opt],[1, self.max_lidar_range_opt]])
            obstacle_points_r=np.array([[0, - self.max_lidar_range_opt],[1, - self.max_lidar_range_opt]])

            k_obs = 0
            
            for k in range(0, self.n_pts_l):

                obs_index = (start_indx_l+k*index_l) % self.scan_beams
                obs_range= fused_ranges[obs_index]

                if obs_range <=self.max_lidar_range_opt:
                
                    if k_obs == 0:
                        obstacle_points_l[0][0]= - obs_range*math.cos(mod_angle_al+k*index_l*data.angle_increment)
                        obstacle_points_l[0][1]= - obs_range*math.sin(mod_angle_al+k*index_l*data.angle_increment)
                    
                    elif k_obs ==1:
                        obstacle_points_l[1][0]= - obs_range*math.cos(mod_angle_al+k*index_l*data.angle_increment)
                        obstacle_points_l[1][1]= - obs_range*math.sin(mod_angle_al+k*index_l*data.angle_increment)
                    
                    else: 
                        x_obs = - obs_range*math.cos(mod_angle_al+k*index_l*data.angle_increment)
                        y_obs= - obs_range*math.sin(mod_angle_al+k*index_l*data.angle_increment)
                        obstacle_points_l = np.vstack((obstacle_points_l,np.array([x_obs, y_obs])))

                    k_obs += 1


            k_obs = 0

            for k in range(0,self.n_pts_r):
            
                obs_index = (start_indx_r+k*index_r) % self.scan_beams
                obs_range= fused_ranges[obs_index]

                if obs_range <=self.max_lidar_range_opt:
                
                    if k_obs == 0:
                        obstacle_points_r[0][0]= - obs_range*math.cos(mod_angle_br+k*index_r*data.angle_increment)
                        obstacle_points_r[0][1]= - obs_range*math.sin(mod_angle_br+k*index_r*data.angle_increment)
                    
                    elif k_obs== 1:
                        obstacle_points_r[1][0]= - obs_range*math.cos(mod_angle_br+k*index_r*data.angle_increment)
                        obstacle_points_r[1][1]= - obs_range*math.sin(mod_angle_br+k*index_r*data.angle_increment)
                        
                    else:
                        x_obs = - obs_range*math.cos(mod_angle_br+k*index_r*data.angle_increment)
                        y_obs = - obs_range*math.sin(mod_angle_br+k*index_r*data.angle_increment)
                        obstacle_points_r = np.vstack((obstacle_points_r,np.array([x_obs, y_obs])))
        
                    k_obs += 1
        

            alpha = 1-math.exp(-dt/self.tau)
            

            wl, wr = self.getWalls(obstacle_points_l, obstacle_points_r, self.wl0, self.wr0, alpha)

            self.wl0 = wl
            self.wr0 = wr 
            
            dl = 1/math.sqrt(np.dot(wl.T,wl))
            dr = 1/math.sqrt(np.dot(wr.T,wr))

            wl_h = wl*dl 
            wr_h = wr*dr 

            self.marker.header.frame_id = "base_link"
            self.marker.header.stamp = rospy.Time.now() 
            self.marker.type = Marker.LINE_LIST
            self.marker.id = 0
            self.marker.action= Marker.ADD
            self.marker.scale.x = 0.1
            self.marker.color.a = 1.0
            self.marker.color.r = 0.5
            self.marker.color.g = 0.5
            self.marker.color.b = 0.0
            self.marker.pose.orientation.w = 1

            self.marker.lifetime=rospy.Duration(0.1)
    

            self.marker.points = []
        
            line_len = 1
            self.marker.points.append(Point(dl*(-wl_h[0]-line_len*wl_h[1]), dl*(-wl_h[1]+line_len*wl_h[0]) , 0))
            self.marker.points.append(Point(dl*(-wl_h[0]+line_len*wl_h[1]), dl*(-wl_h[1]-line_len*wl_h[0]) , 0))
            self.marker.points.append(Point(dr*(-wr_h[0]-line_len*wr_h[1]), dr*(-wr_h[1]+line_len*wr_h[0]) , 0))
            self.marker.points.append(Point(dr*(-wr_h[0]+line_len*wr_h[1]), dr*(-wr_h[1]-line_len*wr_h[0]) , 0))
            self.marker.points.append(Point(0, 0 , 0))
            self.marker.points.append(Point(line_len*math.cos(heading_angle), line_len*math.sin(heading_angle), 0))
   
            self.marker_pub.publish(self.marker)

        
            if self.vel >= 0.01 or self.vel <= -0.01:

                d_tilde= dl-dr - self.CenterOffset
                d_tilde_dot=self.vel*(wl_h[0]-wr_h[0])
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde+self.k_d*d_tilde_dot))/((self.vel**2)*(-wl_h[1]+wr_h[1])))
        
            else:
                delta_d = 0
            

            if delta_d >=self.max_steering_angle:
                delta_d=self.max_steering_angle
            elif delta_d<=-self.max_steering_angle:
                delta_d =-self.max_steering_angle
        
            min_distance = min(fused_ranges[-sec_len+int(self.scan_beams/2):sec_len+int(self.scan_beams/2)])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)
            
            velocity=velocity_scale*self.vehicle_velocity

            if velocity <= self.velocity_zero:

                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()

                t = rospy.Time.from_sec(time.time())
                self.stopped_time = t.to_sec() - self.time_ref

                if self.stopped_time >= self.stop_time1:
                    self.drive_state = "backup"
                    self.time_ref = 0.0
                    self.yaw0 = self.yaw
                    self.turn_angle = np.argmax(mod_ranges)*(2*math.pi/len(mod_ranges)) - math.pi

            else:
                self.time_ref = 0.0
        
        elif self.drive_state == "backup":

            self.dtheta =  self.yaw-self.yaw0
            
            if abs(self.dtheta) >1.0:

                if np.sign(self.dtheta) != np.sign(self.turn_angle):
                    if self.dtheta < 0:
                        self.dtheta = self.dtheta + 4*math.pi
                    else:
                        self.dtheta = self.dtheta - 4*math.pi       

            min_distance = min(fused_ranges[0:sec_len])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)

            delta_d = - np.sign(self.turn_angle)*self.max_steering_angle
            velocity= - velocity_scale*self.turn_velocity


            if abs(self.dtheta) >= abs(self.turn_angle/2.0): 
                self.drive_state = "turn"
                self.time_ref = 0.0
 

            elif - velocity <= self.velocity_zero:
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                    if self.stopped_time >= self.stop_time2: 
                        self.drive_state = "turn"
                        self.time_ref = 0.0
            else:
                 self.time_ref = 0            

        else:

            min_distance = min(fused_ranges[-sec_len+int(self.scan_beams/2):sec_len+int(self.scan_beams/2)])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)

            delta_d =  np.sign(self.turn_angle)*self.max_steering_angle
            velocity = velocity_scale*self.turn_velocity

            self.dtheta =  self.yaw-self.yaw0
            
            if abs(self.dtheta) > 1.0:

                if np.sign(self.dtheta) != np.sign(self.turn_angle):
                    if self.dtheta < 0:
                        self.dtheta = self.dtheta + 4*math.pi
                    else:
                        self.dtheta = self.dtheta - 4*math.pi

            if abs(self.dtheta) >= abs(self.turn_angle):
                delta_d = 0.0
                velocity = 0.0 
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                    self.stopped_time = 0.0
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                if self.stopped_time >= self.stop_time2: 
                    self.drive_state = "normal"
                    self.time_ref = 0.0
                    self.wl0 = np.array([0.0, -1.0])
                    self.wr0 = np.array([0.0, 1.0])

            
            elif velocity <= self.velocity_zero:
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                    if self.stopped_time >= 1.0: 
                        self.drive_state = "backup"
                        self.time_ref = 0.0
                        self.yaw0 = self.yaw
                        self.turn_angle = np.argmax(mod_ranges)*(2*math.pi/len(mod_ranges)) - math.pi

            else:
                self.time_ref = 0.0
                          
            
        # Publish to driver topic
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = delta_d
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x
        #self.yaw_old = self.yaw
        self.yaw = 2*np.arctan2(odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w)


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
