#define _USE_MATH_DEFINES //M_PI

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h" //receive msgs from lidar
#include "sensor_msgs/Imu.h" //receive msgs from Imu
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h" //plot marker line

#include "sensor_msgs/Image.h" // ros image

//CV
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/core.hpp"
// #include "opencv2/opencv.hpp"
// #include "image_transport/image_transport.h"
// #include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/Image.h"
// #include "sensor_msgs/CameraInfo.h"
// #include "librealsense2/rs.hpp"



#include <tf/tf.h> //Quaternions

#include "ackermann_msgs/AckermannDriveStamped.h" //Ackermann Steering

#include "nav_msgs/Odometry.h" //Odometer


#include <stdio.h>
#include <math.h> //cosf
#include <cmath> //M_PI, round
#include <sstream>
#include <algorithm>

#include <QuadProg++.hh>
// #include <iomanip>

// std_msgs::String msg;
// std::stringstream ss;

class GapBarrier {
	private:
		ros::NodeHandle nf;


		//Subscriptions
		ros::Subscriber lidar;
		ros::Subscriber image, info, confidence; 
		ros::Subscriber imu;
		ros::Subscriber mux;
		ros::Subscriber odom;


		
		//Publications
		ros::Publisher lidar_pub;
		ros::Publisher marker_pub;
		ros::Publisher driver_pub;

		
		//topics
		std::string depth_image_topic, depth_info_topic, cv_ranges_topic, depth_index_topic, 
		depth_points_topic,lidarscan_topic, drive_topic, odom_topic, mux_topic, imu_topic;

		//time
		double current_time = ros::Time::now().toSec();
		double prev_time = current_time;
		double time_ref = 0.0; 
		double heading_beam_angle;

		//lidar-preprocessing
		int scan_beams; double right_beam_angle, left_beam_angle;
		int ls_str, ls_end, ls_len_mod, ls_len_mod2; double ls_fov, angle_cen, ls_ang_inc;
		double max_lidar_range, safe_distance;

		//obstacle point detection
		std::string drive_state; 
		double angle_bl, angle_al, angle_br, angle_ar;
		int n_pts_l, n_pts_r; double max_lidar_range_opt;

		//walls
		double tau;
		std::vector<double> wl0; std::vector<double> wr0;
		int optim_mode;


		//markers
		visualization_msgs::Marker marker;

		//steering & stop time
		double vel;
		double CenterOffset, wheelbase;
		double stop_distance, stop_distance_decay;
		double k_p, k_d;
		double max_steering_angle;
		double vehicle_velocity; double velocity_zero;
		
		double stopped_time;
		double stop_time1, stop_time2;

		double yaw0, dtheta; double turn_angle; 
		double turn_velocity;

		//imu
		double imu_roll, imu_pitch, imu_yaw;


		//mux
		int nav_mux_idx; int nav_active; 

		//odom
		double yaw;


		//camera and cv
		int use_camera;
		// bool intrinsics_defined;
		// rs2_intrinsics intrinsics;
		// sensor_msgs::Image cv_image_data;
		// bool cv_image_data_defined; 
		

	public:

		GapBarrier(){

			nf = ros::NodeHandle("~");
			// topics	
			nf.getParam("depth_image_topic", depth_image_topic);
			nf.getParam("depth_info_topic", depth_info_topic);
			nf.getParam("cv_ranges_topic", cv_ranges_topic);
			nf.getParam("depth_index_topic", depth_index_topic);
			nf.getParam("depth_points_topic", depth_points_topic);
			nf.getParam("scan_topic", lidarscan_topic);
			nf.getParam("nav_drive_topic", drive_topic);
			nf.getParam("odom_topic", odom_topic);
			nf.getParam("mux_topic", mux_topic);
			nf.getParam("imu_topic", imu_topic);



			//lidar params
			nf.getParam("scan_beams", scan_beams);
			nf.getParam("right_beam_angle", right_beam_angle);
			nf.getParam("left_beam_angle", left_beam_angle);
			nf.getParam("scan_range", max_lidar_range);
			nf.getParam("safe_distance", safe_distance);

			//lidar init
			ls_ang_inc = 2*M_PI/scan_beams;
			ls_str = int(round(scan_beams*right_beam_angle/(2*M_PI)));
			ls_end = int(round(scan_beams*left_beam_angle/(2*M_PI)));
			ls_len_mod = ls_end-ls_str+1;
			ls_fov = ls_len_mod*ls_ang_inc;
			angle_cen = ls_fov/2;
			ls_len_mod2 = 0;	


			//obstacle point detection
			drive_state = "normal";
			nf.getParam("angle_bl", angle_bl);
			nf.getParam("angle_al", angle_al);
			nf.getParam("angle_br", angle_br);
			nf.getParam("angle_ar", angle_ar);
			nf.getParam("n_pts_l", n_pts_l);
			nf.getParam("n_pts_r", n_pts_r);
			nf.getParam("max_lidar_range_opt", max_lidar_range_opt);
			nf.getParam("heading_beam_angle", heading_beam_angle);

			//walls
			nf.getParam("tau", tau); 
			wl0 = {0.0, -1.0}; wr0 = {0.0, 1.0};
			nf.getParam("optim_mode", optim_mode);


			//steering init
			nf.getParam("CenterOffset", CenterOffset);
			nf.getParam("wheelbase", wheelbase);
			nf.getParam("stop_distance", stop_distance);
			nf.getParam("stop_distance_decay", stop_distance_decay);
			nf.getParam("k_p", k_p);
			nf.getParam("k_d", k_d);
			nf.getParam("max_steering_angle", max_steering_angle);
			nf.getParam("vehicle_velocity", vehicle_velocity);
			nf.getParam("velocity_zero",velocity_zero);
			nf.getParam("turn_velocity", turn_velocity);


			vel = 0.0;

			//timing
			nf.getParam("stop_time1", stop_time1);
			nf.getParam("stop_time2", stop_time2);
			stopped_time = 0.0;

			//camera
			nf.getParam("use_camera", use_camera);


			//imu init
			yaw0 = 0.0; dtheta = 0.0;

			//mux init
			nf.getParam("nav_mux_idx", nav_mux_idx);
			nav_active = 0;

			//cv



			//subscriptions
			lidar = nf.subscribe("/scan",1, &GapBarrier::lidar_callback, this);
			imu = nf.subscribe(imu_topic,1, &GapBarrier::imu_callback, this);
			mux = nf.subscribe(mux_topic,1, &GapBarrier::mux_callback, this);
			odom = nf.subscribe(odom_topic,1, &GapBarrier::odom_callback, this);




			//publications
			lidar_pub = nf.advertise<std_msgs::Int32MultiArray>("chatter", 1000);
			marker_pub = nf.advertise<visualization_msgs::Marker>("wall_markers",2);
			driver_pub = nf.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);


			// if(use_camera){
				


			// 	nf.subscribe(depth_image_topic, 1, &GapBarrier::imageDepth_callback, this);
			// 	// nf.subscribe(depth_info_topic, 1, &GapBarrier::)

			// }

		}



		/// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

		// void publish_lidar(std::vector<int> data2){


		// 	std_msgs::Int32MultiArray lidar_msg;
		// 	lidar_msg.data.clear();

		// 	for(int i =0; i < int(data2.size()); ++i){
		// 		lidar_msg.data.push_back(int(data2[i]));
		// 	}

		// 	lidar_pub.publish(lidar_msg);
		// }

		int equiv_sign(double qt){
			if(qt < 0) return -1;
			else if (qt == 0 ) return 0;
			else return 1;
		}


		int arg_max(std::vector<double> ranges){

			int idx = 0;

			for(int i =1; i < int(ranges.size()); ++i){
				if(ranges[idx] < ranges[i]) idx = i;
			}

			return idx;


		}


		std::string getOdom() const { return odom_topic; }
		int getRightBeam() const { return right_beam_angle;}
		std::string getLidarTopic() const { return lidarscan_topic;}


		/// ---------------------- MAIN FUNCTIONS ----------------------

		// void imageDepth_callback(const sensor_msgs::ImageConstPtr & data){
			
		// 	if(intrinsics_defined){

		// 		cv_image_data = *data;
		// 		cv_image_data_defined = true;
		// 	}


		// }

		void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
			vel = odom_msg->twist.twist.linear.x;
			yaw = 2*atan2(odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
			// ROS_INFO("%.3f", vel);
		}

		void mux_callback(const std_msgs::Int32MultiArrayConstPtr& data){nav_active = data->data[nav_mux_idx]; }

		void imu_callback(const sensor_msgs::Imu::ConstPtr& data){

				tf::Quaternion myQuaternion(
				data->orientation.x,
				data->orientation.y,
				data->orientation.z,
				data->orientation.w);
			
			tf::Matrix3x3 m(myQuaternion);
			m.getRPY(imu_roll, imu_pitch, imu_yaw);
			// ROS_INFO("ROLL: %.3f, PITCH: %.3f, YAW: %.3f", imu_roll, imu_pitch, imu_yaw);

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


		void getWalls(std::vector<std::vector<double>> obstacle_points_l, std::vector<std::vector<double>> obstacle_points_r,
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

				// ROS_INFO("%f, %f", wl[0], wl[1]);


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

				// ROS_INFO("%f, %f", wl[0], wl[1]);

				// std::stringstream ss; 

				// for(int i =0; i < n; ++i){
				// 	for(int j=0; j < n; ++j)
				// 	ss << Gr[i][j] << " ";
				// }

				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

			}
			else{
				quadprogpp::Matrix<double> G,CE,CI;
				quadprogpp::Vector<double> gi0, ce0, ci0, x;

				// char ch;
				int n_obs_l = obstacle_points_l.size(); int n_obs_r = obstacle_points_r.size();
				
				
				int n,m,p;
				n = 3; m = 0; p = n_obs_l + n_obs_r + 2;

				G.resize(n,n);
				{
					// std::istringstream is("1.0, 0.0, 0.0,"
					// 						"0.0, 1.0, 0.0,"
					// 						"0.0, 0.0, 0.0001");

					// for(int i =0; i < n; ++i)
					// 	for(int j =0; j < n-1; ++j)
					// 		is >> G[i][j] >> ch;


					G[0][1] = G[0][2] = G[1][0] = G[1][2] = G[2][0] = G[2][1] = 0.0;
					G[0][0] = G[1][1] = 1.0;
					G[2][2] = 0.0001;

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

				// ss << wl[0] << " " << wl[1];


				// for(int i =0; i < n; ++i)
				// 	for(int j =0; j < n; ++j) ss << G[i][j] << " ";


				// ss << x[0]/(x[2]-1) << " " << x[1]/(x[2]-1);
				// ss << solve_quadprog(G, gi0, CE, ce0, CI, ci0, x);

				// std_msgs::String msg; msg.data = ss.str();
				// ROS_INFO("%s", msg.data.c_str());
				// msg.data.clear();

			}

		}







		void lidar_callback(const sensor_msgs::LaserScanConstPtr &data){
			// ROS_INFO("hello!\n");
			ls_ang_inc = static_cast<double>(data->angle_increment); 
			scan_beams = int(2*M_PI/data->angle_increment);
			ls_str = int(round(scan_beams*right_beam_angle/(2*M_PI)));
			ls_end = int(round(scan_beams*left_beam_angle/(2*M_PI)));

			//TODO: ADD CAMERA

			// if (!nav_active) {
			// 	drive_state = "normal";
			// 	return;
			// }



			//pre-processing
			std::vector<double> double_data; double value;
			for(int i =0; i < int(data->ranges.size()); ++i){
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

			double min_distance, velocity_scale, delta_d, velocity;
			

			if(drive_state == "normal"){
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

				for(int k = 0; k < n_pts_l; ++k){
					obs_index = (start_indx_l + k*index_l) % scan_beams;

					double obs_range = static_cast<double>(data->ranges[obs_index]);
					

					if(obs_range <= max_lidar_range_opt){


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

				for(int k = 0; k < n_pts_r; ++k){
					obs_index = (start_indx_r+k*index_r) % scan_beams;
					double obs_range = static_cast<double>(data->ranges[obs_index]);

					if(obs_range <= max_lidar_range_opt) {
						if(k_obs == 0){
							obstacle_points_r[0] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment), -obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
						}
						else if(k_obs == 1){
							obstacle_points_r[1] = {-obs_range*cos(mod_angle_br+k*index_r*data->angle_increment),-obs_range*sin(mod_angle_br+k*index_r*data->angle_increment)};
						}
						else{
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

				// ROS_INFO("hello");

				getWalls(obstacle_points_l, obstacle_points_r, wl0, wr0, alpha, wr, wl);

				
				wl0[0] = wl[0]; wl0[1] = wl[1];
				wr0[0] = wr[0], wr0[1] = wr[1];

				// ROS_INFO("%.3f, %.3f", wl[0], wl[1]);

				double dl, dr; 
				double wr_dot, wl_dot; 
				wr_dot = wl_dot = 0;

				for(int i =0; i < 2; ++i){
					wl_dot += wl[i]*wl[i];
					wr_dot += wr[i]*wr[i];
				}

				dl = 1/sqrt(wl_dot); dr = 1/sqrt(wr_dot);

				std::vector<double> wr_h = {wr[0]*dr,wr[1]*dr}; std::vector<double> wl_h = {wl[0]*dl, wl[1]*dl};

				// ROS_INFO("%f, %f", wr_h[0], wr_h[1]);
				
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

				p.x = dl*(-wl_h[0]-line_len*wl_h[1]);	p.y = dl*(-wl_h[1]+line_len*wl_h[0]);	p.z = 0; 
				marker.points.push_back(p);

				p.x = dl*(-wl_h[0]+line_len*wl_h[1]);	p.y = dl*(-wl_h[1]-line_len*wl_h[0]);	p.z = 0;
				marker.points.push_back(p);

				p.x = dr*(-wr_h[0]-line_len*wr_h[1]);	p.y = dr*(-wr_h[1]+line_len*wr_h[0]);	p.z = 0;
				marker.points.push_back(p);

				p.x = dr*(-wr_h[0]+line_len*wr_h[1]);	p.y = dr*(-wr_h[1]-line_len*wr_h[0]);	p.z = 0;
				marker.points.push_back(p);

				p.x = 0; p.y = 0; p.z = 0;
				marker.points.push_back(p);

				p.x = line_len*cosf(heading_angle);	p.y = line_len*sinf(heading_angle);	p.z = 0;
				marker.points.push_back(p);

				marker_pub.publish(marker);


				//Ackermann Steering
				if(vel >= 0.01 || vel <= -0.01){
					double d_tilde, d_tilde_dot;
					d_tilde = dl - dr - CenterOffset;
					d_tilde_dot = vel*(wl_h[0]-wr_h[0]);
					delta_d = atan(wheelbase*(k_p*d_tilde + k_d*d_tilde_dot))/((std::pow(vel,2))*(-wl_h[1]+wr_h[1]));
				}
				else delta_d = 0.0;

				// ROS_INFO("%.3f", delta_d);

				if(delta_d >= max_steering_angle) delta_d = max_steering_angle;
				else if (delta_d <= -max_steering_angle) delta_d = -max_steering_angle;

				min_distance = max_lidar_range + 100; int idx1, idx2;
				idx1 = -sec_len+int(scan_beams/2); idx2 = sec_len + int(scan_beams/2);

				for(int i = idx1; i <= idx2; ++i){
					if(data->ranges[i] < min_distance) min_distance = data->ranges[i];
				}

				velocity_scale = 1 - exp(-std::max(min_distance-stop_distance,0.0)/stop_distance_decay);
				velocity = velocity_scale*vehicle_velocity;

				// ROS_INFO("%.3f", velocity);


				if(velocity <= velocity_zero){

					if(time_ref == 0.0){
						t = ros::Time::now();	
						time_ref = t.toSec();				
					}

					t = ros::Time::now();
					double stopped_time = t.toSec() - time_ref;

					if(stopped_time >= stop_time1){
						drive_state = "backup";
						time_ref = 0.0;
						// yaw
						yaw0 = yaw;
						turn_angle = arg_max(mod_ranges) * (2.0 * M_PI/mod_ranges.size())-M_PI;
						// ROS_INFO("turn_angle: %f", turn_angle);
					}

				}	
				else time_ref = 0.0;
			}
			else if(drive_state == "backup"){

				dtheta = yaw - yaw0;

				if(abs(dtheta) > 1.0){
					if(equiv_sign(dtheta) != equiv_sign(turn_angle)) dtheta += 4*M_PI;
					else dtheta -= 4*M_PI;
				}

				min_distance = *std::min_element(data->ranges.begin(), data->ranges.begin()+sec_len);
				velocity_scale =1-exp(-std::max(min_distance - stop_distance, 0.0)/stop_distance_decay);

				delta_d = - equiv_sign(turn_angle)*max_steering_angle;
				velocity = - velocity_scale * turn_velocity;

				if(abs(dtheta) >=abs(turn_angle/2.0)){
					drive_state = "turn";
					time_ref = 0.0;

				}
				else if(-velocity <= velocity_zero){
					if(time_ref == 0.0){
						t = ros::Time::now();
						time_ref = t.toSec();
					}
					else{
						t=ros::Time::now();
						stopped_time = t.toSec() - time_ref;

						if(stopped_time >= stop_time2){
							drive_state = "turn";
							time_ref = 0.0;
						}
					}
				}
				else time_ref = 0.0;


			}
			else {

				min_distance = *std::min_element(data->ranges.begin()-sec_len+int(scan_beams/2), data->ranges.begin()+sec_len+int(scan_beams/2));
				velocity_scale = 1-exp(-std::max(min_distance-stop_distance, 0.0)/stop_distance_decay);

				delta_d = equiv_sign(turn_angle)*max_steering_angle;
				velocity = velocity_scale*turn_velocity;

				dtheta = yaw - yaw0;

				if(abs(dtheta) > 1.0){

					if(equiv_sign(dtheta) != equiv_sign(turn_angle)){
						if(dtheta <0) dtheta += 4*M_PI;
						else dtheta -= 4*M_PI;
					}
					
				}

				if(abs(dtheta) >= abs(turn_angle)){
					delta_d = 0.0;
					velocity = 0.0;

					if(time_ref == 0.0){
						t = ros::Time::now();
						time_ref = t.toSec();
						stopped_time = 0.0;
					}
					else{
						t = ros::Time::now();
						stopped_time = t.toSec() - time_ref;
					}
					if(stopped_time >= stop_time2){
						drive_state = "normal";
						time_ref = 0.0;
						wl0[0] = 0.0; wl0[1] = -1.0;
						wr0[0] = 0.0; wr0[1] = 1.0;
					}
				}

				else if(velocity <= velocity_zero){
					if(time_ref == 0.0){
						t = ros::Time::now();
						time_ref = t.toSec();
					}
					else{
						t=ros::Time::now();
						stopped_time = t.toSec() - time_ref;

						if(stopped_time >= 1.0){
							drive_state = "backup";
							time_ref = 0.0;
							yaw0 = yaw;
							turn_angle = arg_max(mod_ranges)* (2.0 * M_PI/mod_ranges.size())-M_PI;							
						}
					}
				}
				else time_ref = 0.0;

			}


			ackermann_msgs::AckermannDriveStamped drive_msg; 
			drive_msg.header.stamp = ros::Time::now();
			drive_msg.header.frame_id = "base_link";
			drive_msg.drive.steering_angle = delta_d;
			drive_msg.drive.speed = velocity;

			driver_pub.publish(drive_msg);

		}


};

int main(int argc, char **argv){
		ros::init(argc, argv, "navigation");
		GapBarrier gb;

		while(ros::ok()){
			// std_msgs::String msg;
			// std::stringstream ss;
			// ss  << gb.preprocess_lidar(); 
			// msg.data = ss.str();

			// ROS_INFO("%s", msg.data.c_str());

			// chatter_pub.publish(msg);
			ros::spinOnce();
		}

}


