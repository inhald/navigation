#include "ros/ros.h"
#include <ros/package.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>


//#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
//pi is referred to as M_PI
#include <string>

#include <ctime> 

#include <xtensor/xarray.hpp>

class GapBarrier
{
    private:
    // intialization of topic names
    // There are two steps to getting parameters in roscpp, 
    // 1) declare variable 2) pass the variable and the paramater to the get function to obtain the value


        std::string depth_image_topic;        
        std::string depth_info_topic; 
        std::string cv_ranges_topic;
        std::string depth_index_topic;
        std::string depth_points_topic;
        std::string lidarscan_topic;
        std::string drive_topic;
        std::string odom_topic;
        std::string mux_topic;
        std::string imu_topic;

        int nav_mux_idx;
        
        double max_steering_angle;
        double max_lidar_range;
        double wheelbase;      
        double CenterOffset;
        double k_p;
        double k_d;
        double tau;

        double min_cv_range;
        double max_cv_range;
        double cv_distance_to_lidar;
        double num_cv_sample_rows;
        double num_cv_sample_cols;

        double cv_ground_angle;
        double cv_lidar_range_max_diff;
        double camera_height;
        double cv_real_to_theo_ground_range_ratio;
        double cv_real_to_theo_ground_range_ratio_near_horizon;
        double cv_ground_range_decay_row;
        double cv_pitch_angle_hardcoded;

        double angle_bl;
        double angle_al;
        double angle_br;
        double angle_ar;
        int n_pts_l;
        int n_pts_r;
        double vehicle_velocity;
        double turn_velocity;
        double turn_angle1;
        double turn_angle2;
        double stop_time1;
        double stop_time2;
        int scan_beams;
        double safe_distance;
        double right_beam_angle;
        double left_beam_angle;
        double heading_beam_angle;
        double stop_distance;
        double stop_distance_decay;
        double velocity_zero;
        int optim_mode;
        int use_camera;
        double max_lidar_range_opt;

        int counter;
        double vel;
        double ls_ang_inc;
        int nav_active;



        // Search FOV definition variables
        int ls_str;
        int ls_end;
        int ls_len_mod;
        double ls_fov;
        double angle_cen;
        //double ls_len_mod2
        //std::vector <double> ls_data;

        std::string drive_state;
        double stopped_time;
        double yaw0;
        double dtheta;
        double yaw;
        double imu_roll;
        double imu_pitch;
        double imu_yaw;
        double t;
        double current_time;
        double prev_time;
        double time_ref;
        //DO: add using numpy equivalent data type definitions for
        // wl0
        //wr0

        //rosnode handler used to create publishers and subscribers
        ros::NodeHandle nodeHandler;

        //Publiisher and subscriber data members

        ros::Subscriber lidar;
        ros::Subscriber odom;
        ros::Subscriber mux;
        ros::Subscriber imu;


        ros::Publisher marker_pub;
        //DO: declare marker object
        ros::Publisher drive_pub;

        void imu_callback(const sensor_msgs::ImuConstPtr & data)
        {
            double Imu_msg[4]= {data->orientation.x , data->orientation.y, data->orientation.z, data->orientation.w}; 
            //DO euler from quaterian 
            //imu_roll=
            //imu_pitch=
            //imu_yaw=git 
        }
        void mux_callback(const std_msgs::Int32MultiArrayConstPtr& mux_data)
        {
            nav_active= mux_data->data[nav_mux_idx]; //mux data is of type std::vector<int>
        }
        void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg )
        {
            vel= odom_msg->twist.twist.linear.x; //message types nest within each other
            //DO update yaw value once found numpy equivalent.


        }


        void lidar_callback(const sensor_msgs::LaserScanConstPtr& data ) //msg type is Laser Scan, expecting const ptr
        {
            
        }


    public:
        GapBarrier()
        {
            ros::param::get("~depth_image_topic", depth_image_topic);
            ros::param::get("~depth_info_topic", depth_info_topic);
            ros::param::get("~cv_ranges_topic", cv_ranges_topic);
            ros::param::get("~depth_index_topic",depth_index_topic);
            ros::param::get("~depth_points_topic", depth_points_topic);
            ros::param::get("~scan_topic", lidarscan_topic);
            ros::param::get("~nav_drive_topic",drive_topic);
            ros::param::get("~odom_topic", odom_topic);
            ros::param::get("~mux_topic", mux_topic);
            ros::param::get("~imu_topic", imu_topic);

            ros::param::get("~nav_mux_idx", nav_mux_idx);

            ros::param::get("~max_steering_angle", max_steering_angle);
            ros::param::get("~scan_range", max_lidar_range);
            ros::param::get("~wheelbase",wheelbase);
            ros::param::get("~CenterOffset",CenterOffset);
            ros::param::get("~k_p", k_p);
            ros::param::get("~k_d", k_d);
            ros::param::get("~tau", tau);

            ros::param::get("~min_cv_range", min_cv_range);
            ros::param::get("~max_cv_range", max_cv_range);
            ros::param::get("~cv_distance_to_lidar", cv_distance_to_lidar);
            ros::param::get("~num_cv_sample_rows", num_cv_sample_rows);
            ros::param::get("~num_cv_sample_cols",num_cv_sample_cols);

            ros::param::get("~cv_ground_angle", cv_ground_angle);
            ros::param::get("~cv_lidar_range_max_diff",cv_lidar_range_max_diff);
            ros::param::get("~camera_height",camera_height);
            ros::param::get("~cv_real_to_theo_ground_range_ratio",cv_real_to_theo_ground_range_ratio);
            ros::param::get("~cv_real_to_theo_ground_range_ratio_near_horizon",cv_real_to_theo_ground_range_ratio_near_horizon);
            ros::param::get("~cv_ground_range_decay_row",cv_ground_range_decay_row);
            ros::param::get("~cv_pitch_angle_hardcoded",cv_pitch_angle_hardcoded);

            ros::param::get("~angle_bl",angle_bl);
            ros::param::get("~angle_al",angle_al);
            ros::param::get("~angle_br",angle_br);
            ros::param::get("~angle_ar",angle_ar);
            ros::param::get("~n_pts_l",n_pts_l);
            ros::param::get("~n_pts_r",n_pts_r);
            ros::param::get("~vehicle_velocity",vehicle_velocity);
            ros::param::get("~turn_velocity",turn_velocity);
            ros::param::get("~turn_angle1",turn_angle1);
            ros::param::get("~turn_angle2",turn_angle2);
            ros::param::get("~stop_time1",stop_time1);
            ros::param::get("~stop_time2 ",stop_time2 );
            ros::param::get("~scan_beams",scan_beams);
            ros::param::get("~safe_distance",safe_distance);
            ros::param::get("~right_beam_angle",right_beam_angle);
            ros::param::get("~left_beam_angle",left_beam_angle);
            ros::param::get("~heading_beam_angle",heading_beam_angle);
            ros::param::get("~stop_distance",stop_distance);
            ros::param::get("~stop_distance_decay",stop_distance_decay);
            ros::param::get("~velocity_zero",velocity_zero);
            ros::param::get("~optim_mode",optim_mode);
            ros::param::get("~use_camera",use_camera);
            ros::param::get("~max_lidar_range_opt",max_lidar_range_opt);

            counter=0;
            vel=0;
            ls_ang_inc=2*M_PI/scan_beams;
            nav_active=0;
            
            //DO: add camera setup


            //Lidar FOV definition
            ls_str=int(std::round(scan_beams*right_beam_angle/(2*M_PI)));
            ls_end=int(std::round(scan_beams*left_beam_angle/(2*M_PI)));
            ls_len_mod= ls_str-ls_end+1;
            ls_fov= ls_len_mod*ls_ang_inc;
            angle_cen=ls_fov/2;
            // ls_len_mod2=0;
            // ls_data=[];

            drive_state="normal";
            stopped_time=0;
            yaw0=0;
            dtheta=0.0;
            yaw=0.0;
            imu_roll=0;
            imu_pitch=0;
            imu_yaw=0;
            //t  rospy.Time.from_sec(time.time())
            //self.current_Time=t.to_sec()
            prev_time=current_time;
            time_ref=0.0;
            //DO: add wr0 and wl0 definitons using numpy equivalent array class

            //DO: add subscriber intialzations

            lidar= nodeHandler.subscribe(lidarscan_topic,1, &GapBarrier::lidar_callback,this);
            odom= nodeHandler.subscribe(odom_topic,1, &GapBarrier::odom_callback,this);
            mux= nodeHandler.subscribe(mux_topic,1, &GapBarrier::mux_callback,this);
            imu= nodeHandler.subscribe(imu_topic,1, &GapBarrier::imu_callback,this);

            marker_pub= nodeHandler.advertise<visualization_msgs::Marker>("wall_markers", 2); // <msg type>("topic name", queue size)

            // DO: intialize Marker object

            drive_pub= nodeHandler.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
            

            if(use_camera)
            {

            }
        }
        


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation"); //command line arguments + node name

    while(ros::ok())
    {
    

        ros::spinOnce(); //allow ROS backend to update
    }

    return 0;
}