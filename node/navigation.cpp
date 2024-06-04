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
#include "sensor_msgs/LaserScan.h" //receive msgs from lidar
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Imu.h>


#include <stdio.h>
#include <math.h> //cosf
#include <cmath> //M_PI, round
#include <sstream>
#include <algorithm>

//CV includes
#include <cv_bridge/cv_bridge.h>


//#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
//pi is referred to as M_PI
#include <string>

#include <ctime> 

#include <QuadProg++.hh>

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

        //markers
		visualization_msgs::Marker marker;

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

        void publish_lidar(std::vector<int> data2){


			std_msgs::Int32MultiArray lidar_msg;
			lidar_msg.data.clear();

			for(int i =0; i < int(data2.size()); ++i){
				lidar_msg.data.push_back(int(data2[i]));
			}

			lidar_pub.publish(lidar_msg);
		}

        std::pair <std::vector<std::vector<double>>, std::vector<double>>preprocess_lidar(std::vector<double> ranges){

			std::vector<std::vector<double>> data(ls_len_mod,std::vector<double>(2));
			std::vector<double> data2(100);

			//sets distance to zero for obstacles in safe distance, and max_lidar_range for those that are far.
			for(int i =0; i < ls_len_mod; ++i){
				if(ranges[ls_str+i] <= safe_distance) {data[i][0] = 0; data[i][1] = i*ls_ang_inc-angle_cen;}
				else if(ranges[ls_str+i] <= max_lidar_range) {data[i][0] = ranges[ls_str+i]; data[i][1] = i*ls_ang_inc-angle_cen;}
				else {data[i][0] = max_lidar_range; data[i][1] = i*ls_ang_inc-angle_cen;}
			}


			int k1 = 100; int k2 = 40;
			double s_range = 0; int index1, index2;
			
			//moving window
			for(int i =0; i < k1; ++i){
				s_range = 0;

				for(int j =0; j < k2; ++j){
					index1 = int(i*ranges.size()/k1+j);
					if(index1 >= int(ranges.size())) index1 -= ranges.size();
					
					index2 = int(i*ranges.size()/k1-j);
					if(index2 < 0) index2 += ranges.size();

					s_range += std::min(ranges[index1], max_lidar_range) + std::min(ranges[index2], max_lidar_range);

				}
				data2[i] = static_cast<double>(s_range);
			}

			return std::make_pair(data,data2);
			
		}

        std::pair<int, int> find_max_gap(std::vector<std::vector<double>> proc_ranges){
			int j =0; int str_indx = 0; int end_indx = 0; 
			int str_indx2 = 0; int end_indx2 = 0;
			int range_sum = 0; int range_sum_new = 0;

			/*This just finds the start and end indices of gaps (non-safe distance lidar return)
			then does a comparison to find the largest such gap.*/
			for(int i =0; i < ls_len_mod; ++i){
				if(proc_ranges[i][0] != 0){
					if (j==0){
						str_indx = i;
						range_sum_new = 0;
						j = 1;
					}
					range_sum_new += proc_ranges[i][0];
					end_indx = i;
				}
				if(j==1 && (proc_ranges[i][0] == 0 || i == ls_len_mod - 1)){
					j = 0;

					if(range_sum_new > range_sum){
						end_indx2 = end_indx;
						str_indx2 = str_indx;
						range_sum = range_sum_new;
					}
				}
			}

			return std::make_pair(str_indx2, end_indx2);
		}

        double find_best_point(int start_i, int end_i, std::vector<std::vector<double>> proc_ranges){
			double range_sum = 0;
			double best_heading =0;


			for(int i = start_i; i <= end_i; ++i){
				range_sum += proc_ranges[i][0];
				best_heading += proc_ranges[i][0]*proc_ranges[i][1];

			}

			if(range_sum != 0){
				best_heading /= range_sum;
			}

			return best_heading; 
		}

        void getWalls(std::vector<std::vector<double>> &obstacle_points_l, std::vector<std::vector<double>> &obstacle_points_r,
		std::vector<double> wl0, std::vector<double> wr0, double alpha, std::vector<double> &wr, std::vector<double> &wl){
			if(!optim_mode){
				//right
				quadprogpp::Matrix<double> Gr,CEr,CIr;
				quadprogpp::Vector<double> gr0,ce0r,ci0r,xr;
				int n,m,p; char ch;
				int n_obs_r = obstacle_points_r.size(); int n_obs_l = obstacle_points_l.size();


				//left
				n = 2; m = 0; p = n_obs_l;
				quadprogpp::Matrix<double> Gl, CEl, CIl;
				quadprogpp::Vector<double> gl0, ce0l, ci0l, xl;


				//left matrices
				Gl.resize(n,n);
				{
					std::istringstream is("1.0, 0.0,"
														"0.0, 1.0 ");

					for(int i=0; i < n; ++i)
						for(int j=0; j < n; ++j)
							is >> Gl[i][j] >> ch;
				}
				gl0.resize(n);
				{
					for(int i =0; i < int(wl0.size()); ++i) gl0[i] = wl0[i] * (alpha-1);
				}
				CEl.resize(n,m);
				{
					CEl[0][0] = 0.0;
					CEl[1][0] = 0.0;
				}
				ce0l.resize(m);

				CIl.resize(n,p);
				{
					for(int i=0; i < p; ++i){
						CIl[0][i] = -obstacle_points_l[i][0];
						CIl[1][i] = -obstacle_points_l[i][1]; 
					}
				}
				ci0l.resize(p);
				{
					for(int i =0; i < p; ++i) ci0l[i] = -1.0;
				}

				xl.resize(n);
				// std::stringstream ss;
				solve_quadprog(Gl, gl0, CEl, ce0l, CIl, ci0l, xl);
				wl[0] = xl[0]; wl[1] = xl[1];
				// ss << xl[0] << " " << xl[1];
				// for(int j =0; j < int(obstacle_points_l.size()); ++j){
				// 	ss << obstacle_points_l[j][0] << " " << obstacle_points_l[j][1];
				// }
				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();


				p = n_obs_r;
				//right matrices
				Gr.resize(n,n);
				{
					std::istringstream is("1.0, 0.0,"
													"0.0, 1.0 ");

					for(int i =0; i < n; ++i)
						for(int j=0; j < n; ++j)
							is >> Gr[i][j] >> ch;

				}

				gr0.resize(n);
				{
					for(int i = 0; i < int(wr0.size()); ++i) gr0[i] = wr0[i] * (alpha-1);
				}

				CEr.resize(n,m);
				{
					CEr[0][0] = 0.0;
					CEr[1][0] = 0.0;
				}
				ce0r.resize(m);

				
				CIr.resize(n,p);
				{
						for(int i =0; i < p; ++i){
							CIr[0][i] = -obstacle_points_r[i][0];
							CIr[1][i] = -obstacle_points_r[i][1];
						}
				}

				ci0r.resize(p);
				{
					for(int i =0; i < p; ++i) ci0r[i] = -1.0;
				}

				xr.resize(n);
				solve_quadprog(Gr, gr0, CEr, ce0r, CIr, ci0r, xr);
				// ss << xr[0] << " " << xr[1];
				wr[0] = xr[0]; wr[1] = xr[1]; 


				// for(int i =0; i < n; ++i){
				// 	for(int j=0; j < p; ++j){
				// 		ss << obstacle_points_r[i][j] << " ";
				// 	}
				// 	ss << "\n";
				// }

				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

			}
			else{
				quadprogpp::Matrix<double> G,CE,CI;
				quadprogpp::Vector<double> gi0, ce0, ci0, x;

				char ch;
				int n_obs_l = obstacle_points_l.size(); int n_obs_r = obstacle_points_r.size();
				
				
				int n,m,p;
				n = 3; m = 0; p = n_obs_l + n_obs_r + 2;

				G.resize(n,n);
				{
					std::istringstream is("1.0, 0.0, 0.0,"
													"0.0, 1.0, 0.0,"
																	"0.0, 0.0, 0.0001 ");

					for(int i =0; i < n; ++i)
						for(int j =0; j < n; ++j)
							is >> G[i][j] >> ch;

				}
				gi0.resize(n);
				{
					for(int i =0; i < n; ++i) gi0[i] = 0.0;
				}

				CE.resize(n,m);
				{
					CE[0][0] = 0.0;
					CE[1][0] = 0.0;
					CE[2][0] = 0.0;
				}
				ce0.resize(m);

				CI.resize(n,p);
				{
					for(int i =0; i < n_obs_r; ++i){
						CI[0][i] = obstacle_points_r[i][0];
						CI[1][i] = obstacle_points_r[i][1];
						CI[2][i] = 1.0;
					}

					for(int i = n_obs_r; i < n_obs_l + n_obs_r; ++i){
						CI[0][i] = -obstacle_points_l[i-n_obs_r][0];
						CI[1][i] = -obstacle_points_l[i-n_obs_r][1];
						CI[2][i] = -1.0;
					}

					CI[0][n_obs_l+n_obs_r] = 0.0; CI[1][n_obs_l+n_obs_r] = 0.0; CI[2][n_obs_l+n_obs_r] = 1.0;
					CI[0][n_obs_l+n_obs_r+1] = 0.0; CI[1][n_obs_l+n_obs_r+1] = 0.0; CI[2][n_obs_l+n_obs_r+1] = -1.0;

				}
				ci0.resize(p);
				{
					for(int i =0; i < n_obs_r+n_obs_l; ++i){
						ci0[i] = -1.0;
					}
					
					ci0[n_obs_r+n_obs_l] = 0.9; ci0[n_obs_r+n_obs_l+1] = 0.9;
				}
				x.resize(n);

				solve_quadprog(G, gi0, CE, ce0, CI, ci0, x);

				wr[0] = (x[0]/(x[2]-1)); wr[1] = (x[1]/(x[2]-1));

				wl[0] = (x[0]/(x[2]+1)); wl[1] = (x[1]/(x[2]+1));
				

				// std::stringstream ss;

				// for(int i =0 ;i < n; ++i){
				// 	for(int j =0; j < n; ++j)
				// 		ss << G[i][j] << " ";
				// 	ss << "\n";
				// }



				// ss << x[0]/(x[2]-1) << " " << x[1]/(x[2]-1);
				// ss << solve_quadprog(G, gi0, CE, ce0, CI, ci0, x);

				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

			}

		}

        void lidar_callback(const sensor_msgs::LaserScanConstPtr& data ) //msg type is Laser Scan, expecting const ptr
        {
            // ROS_INFO("hello!\n");
			ls_ang_inc = static_cast<double>(data->angle_increment); 
			scan_beams = int(2*M_PI/data->angle_increment);
			ls_str = int(round(scan_beams*right_beam_angle/(2*M_PI)));
			ls_end = int(round(scan_beams*left_beam_angle/(2*M_PI)));

			//TODO: ADD CAMERA


			//pre-processing
			std::vector<double> double_data; double value;
			for(int i =0; i < int(data->ranges.size()); ++i)
            {
				value = static_cast<double>(data->ranges[i]);
				double_data.push_back(value);
			}
			// std::transform(data->ranges.begin(), data->ranges.end(), std::back_inserter(double_data),[](float value)
			// {return static_cast<double>(value); });

			std::pair<std::vector<std::vector<double>>, std::vector<double>> lidar_preprocess = preprocess_lidar(double_data);


			std::vector<std::vector<double>> proc_ranges = lidar_preprocess.first;
			std::vector<double> mod_ranges = lidar_preprocess.second;
			// publish_lidar(mod_ranges);

			// std::stringstream ss; 
			// for(int i =0; i < ls_len_mod ;++i){
			// 	for(int j =0; j < 2; ++j){
			// 		ss << proc_ranges[i][j] << " ";
			// 	}
			// }
			// std_msgs::String msg; msg.data = ss.str();
			// ROS_INFO("%s", msg.data.c_str());


			//TODO: ADD TIME STUFF HERE
			ros::Time t = ros::Time::now();
			current_time = t.toSec();
			double dt = current_time - prev_time;
			prev_time = current_time;
			int sec_len = int(heading_beam_angle/data->angle_increment);

			// std::stringstream ss; 
			// ss << str_indx << " " << end_indx;
			// ss << dt;
			// std_msgs::String msg; msg.data = ss.str();
			// ROS_INFO("%s", msg.data.c_str());
			

			if(drive_state == "normal")
            {
				int str_indx, end_indx, index_l, index_r, start_indx_l, start_indx_r; double heading_angle;
				std::pair<int,int> max_gap = find_max_gap(proc_ranges);
				str_indx = max_gap.first; end_indx = max_gap.second;



				heading_angle= find_best_point(str_indx, end_indx, proc_ranges);

				// std::stringstream ss; 
				// ss << str_indx << " " << end_indx;
				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());

				index_l = int(round((angle_bl-angle_al)/(data->angle_increment*n_pts_l)));
				index_r = int(round((angle_ar-angle_br)/(data->angle_increment*n_pts_r)));

				double mod_angle_al = angle_al + heading_angle;

				if(mod_angle_al > 2*M_PI) mod_angle_al -= 2*M_PI;
				else if (mod_angle_al < 0) mod_angle_al += 2*M_PI;

				double mod_angle_br = angle_br + heading_angle;

				if(mod_angle_br > 2*M_PI) mod_angle_br -= 2*M_PI;
				else if (mod_angle_br < 0) mod_angle_br += 2*M_PI;

				start_indx_l = int(round(mod_angle_al/data->angle_increment));
				start_indx_r = int(round(mod_angle_br/data->angle_increment));

				/// ----- CHECKED UNTIL HERE (GIVING AROUND SAME VALUES) ----

				std::vector<std::vector<double>> obstacle_points_l;
				obstacle_points_l.push_back({0, max_lidar_range_opt});
				obstacle_points_l.push_back({1, max_lidar_range_opt}); 
				// obstacle_points_l[0][0] = 0;
				// obstacle_points_l[0][1] = max_lidar_range_opt;
				// obstacle_points_l[1][0] = 1;
				// obstacle_points_l[1][1] = max_lidar_range_opt;

				std::vector<std::vector<double>> obstacle_points_r;
				obstacle_points_r.push_back({0, -max_lidar_range_opt});
				obstacle_points_r.push_back({1, -max_lidar_range_opt});
				// obstacle_points_r[0][0] = 0;
				// obstacle_points_r[0][1] = -max_lidar_range_opt;
				// obstacle_points_r[1][0] = 1;
				// obstacle_points_r[1][1] = -max_lidar_range_opt;


				// std::stringstream ss;
				// ss << mod_angle_al;
				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

				int k_obs = 0; int obs_index;

				double x_obs, y_obs;

				for(int k = 0; k < n_pts_l; ++k)
                {
					obs_index = (start_indx_l + k*index_l) % scan_beams;

					double obs_range = static_cast<double>(data->ranges[obs_index]);
					

					if(obs_range <= max_lidar_range_opt)
                    {


						if(k_obs == 0){
							obstacle_points_l[0] = {-obs_range*cos(mod_angle_al+k*index_l*data->angle_increment), -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment) };
						}
						else if (k_obs == 1){
							obstacle_points_l[1] = {-obs_range*cos(mod_angle_al+k*index_l*data->angle_increment), -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment) };
						}
						else{
							x_obs = -obs_range*cos(mod_angle_al+k*index_l*data->angle_increment);
							y_obs = -obs_range*sin(mod_angle_al+k*index_l*data->angle_increment);

							std::vector<double> obstacles = {x_obs, y_obs};
							obstacle_points_l.push_back(obstacles);
						}

						k_obs+=1;
					}

				}
				k_obs = 0;

				for(int k = 0; k < n_pts_r; ++k)
                {
					obs_index = (start_indx_r+k*index_r) % scan_beams;
					double obs_range = static_cast<double>(data->ranges[obs_index]);

					if(obs_range <= max_lidar_range_opt) 
                    {
						if(k_obs == 0)
                        {
							obstacle_points_r[0] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment), -obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
						}
						else if(k_obs == 1)
                        {
							obstacle_points_r[1] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment),-obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
						}
						else
                        {
							x_obs = -obs_range*cos(mod_angle_br+k*index_r*data->angle_increment);
							y_obs = -obs_range*sin(mod_angle_br+k*index_r*data->angle_increment);

							// std::stringstream ss;
							// std_msgs::String msg;
							std::vector<double> obstacles = {x_obs, y_obs};
							obstacle_points_r.push_back(obstacles);
							// ss << x_obs << " " << y_obs;
							// ss << " ";
							// ss << obstacle_points_r[k_obs][0] << " " << obstacle_points_r[k_obs][1]; msg.data = ss.str();
							// ROS_INFO("%s", msg.data.c_str()); msg.data.clear();
						}

						k_obs += 1;
					}
				}


				double alpha = 1-exp(-dt/tau);

				std::vector<double> wl = {0.0, 0.0};
				std::vector<double> wr = {0.0, 0.0};

				getWalls(obstacle_points_l, obstacle_points_r, wl0, wr0, alpha, wr, wl);

				// ERROR HERE
				
				wl0 = wl;
				wr0 = wr;

				double dl, dr; 
				double wr_dot, wl_dot; 
				wr_dot = wl_dot = 0;

				for(int i =0; i < 2; ++i)
                {
					wl_dot += wl[i]*wl[i];
					wr_dot += wr[i]*wr[i];
				}

				dl = 1/sqrt(wl_dot); dr = 1/sqrt(wr_dot);

				std::vector<double> wr_h = {wr[0]*dr,wr[1]*dr}; std::vector<double> wl_h = {wl[0]*dl, wl[1]*dl};
				
				// std::stringstream ss;
				// ss << wl_h[0] << " " << wl_h[1];
				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();


				

				marker.header.frame_id = "base_link";
				marker.header.stamp = ros::Time::now();
				marker.type = visualization_msgs::Marker::LINE_LIST;
				marker.id = 0; 
				marker.action = visualization_msgs::Marker::ADD;
				marker.scale.x = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.5; 
				marker.color.g = 0.5;
				marker.color.b = 0.0;
				marker.pose.orientation.w = 1;
				
				marker.lifetime = ros::Duration(0.1);

				int line_len = 1;
				geometry_msgs::Point p;
				marker.points.clear();
				p.x = dl*(-wl_h[0]-line_len*wl_h[1]);	p.y = dl*(-wl_h[1]+line_len*wl_h[0]);	
                p.z = 0; 
				marker.points.push_back(p);

				p.x = dl*(-wl_h[0]+line_len*wl_h[1]);	p.y = dl*(-wl_h[1]-line_len*wl_h[0]);	
                p.z = 0;
				marker.points.push_back(p);

				p.x = dr*(-wr_h[0]-line_len*wr_h[1]);	p.y = dr*(-wr_h[1]+line_len*wr_h[0]);	
                p.z = 0;
				marker.points.push_back(p);

				p.x = dr*(-wr_h[0]+line_len*wr_h[1]);	p.y = dr*(-wr_h[1]-line_len*wr_h[0]);	
                p.z = 0;
				marker.points.push_back(p);

				p.x = 0; p.y = 0; p.z = 0;
				marker.points.push_back(p);

				p.x = line_len*cosf(heading_angle);	p.y = line_len*sinf(heading_angle);	p.z = 0;
				marker.points.push_back(p);

				marker_pub.publish(marker);


            
            }
        }



    public:
        GapBarrier()
        {
            //lidar init
            drive_state = "normal";
            ls_ang_inc = 2*M_PI/scan_beams;
			ls_str = int(round(scan_beams*right_beam_angle/(2*M_PI)));
			ls_end = int(round(scan_beams*left_beam_angle/(2*M_PI)));
			ls_len_mod = ls_end-ls_str+1;
			ls_fov = ls_len_mod*ls_ang_inc;
			angle_cen = ls_fov/2;
			ls_len_mod2 = 0;	


            //walls
            wl0 = {0.0, -1.0}; wr0 = {0.0, 1.0};



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
    wf=GapBarrier();
    ros::Rate rate(10); //ensure node runs at 10Hz
    //TODO convert rospy.sleep(0.1);
    ros::spin();
    rate.sleep(); // sleep for as long as needed to maintain 10 Hz
    
    return 0;
}