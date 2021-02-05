
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <franka_msgs/FrankaState.h>

geometry_msgs::Pose pose_tracker_;
Eigen::Matrix4d tracker_offset_inv_, T_0to8_calib_, T_0tolink8Result_, T_0to8_, T_point_;
bool flag_calibration = false;
std::vector<geometry_msgs::Pose>  vec_point_;
std::vector<geometry_msgs::PoseArray>  vec_traj_;
geometry_msgs::PoseArray pose_array_tmp_;
std::vector<int>  vec_task_type_;
int state_ = 0;
int count_play_task_ = 0;
int count_play_point_ = 0;
int count_play_traj_ = 0;


void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	pose_tracker_ = msg->pose.pose;
}

bool callback_calibration(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 1;
  return true;
}

bool callback_save_point(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 2;
  return true;
}
bool callback_start_traj(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 3;
  return true;
}

bool callback_end_traj(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 4;
  return true;
}

bool callback_clear_task(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 5;
  return true;
}

bool callback_play_task(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 6;
  return true;
}

Eigen::Matrix4d pose_interp(double t, double t1, double t2, Eigen::Matrix4d const& M1, Eigen::Matrix4d const& M2) {
  // assume here t1 <= t <= t2
  double alpha = 0.0;
  if (t2 != t1) alpha = (t - t1) / (t2 - t1);

  Eigen::Quaterniond rot1(M1.block<3,3>(0,0));
  Eigen::Quaterniond rot2(M2.block<3,3>(0,0));

  Eigen::Vector3d trans1 = M1.block<3,1>(0,3);
  Eigen::Vector3d trans2 = M2.block<3,1>(0,3);

  Eigen::Matrix4d result;
  result.block<3,1>(0,3) = (1.0 - alpha) * trans1 + alpha * trans2;
  result.block<3,3>(0,0) = rot1.slerp(alpha, rot2).toRotationMatrix();

  return result;
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "magic_pen_node");
	ros::NodeHandle nh;

  double rate_50Hz = 50.0;
  double dt = 1.0/rate_50Hz;
  double time_to_point = 5.0;
  int step_play = 0;
  bool fist_time = true;
	ros::Rate r_50HZ(rate_50Hz);
	// //Subscriber
	ros::Subscriber sub_tracker_pose = nh.subscribe("/vive/LHR_042ED232_pose", 1, &callback_tracker);
 //  // ros::Subscriber sub_curr_pos_ = nh.subscribe("/franka_state_controller/franka_states", 1, &franka_bridge::callback_curr_pose, this);
 //  //Service Server
  ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);
  ros::ServiceServer server_save_point = nh.advertiseService("/save_point", &callback_save_point);
  ros::ServiceServer server_start_traj = nh.advertiseService("/start_traj", &callback_start_traj);
  ros::ServiceServer server_end_traj = nh.advertiseService("/end_traj", &callback_end_traj);
  ros::ServiceServer server_clear_task = nh.advertiseService("/clear_task", &callback_clear_task);
  ros::ServiceServer server_play_task = nh.advertiseService("/play_task", &callback_play_task);

  T_0tolink8Result_ = Eigen::Matrix4d::Identity();
  T_point_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_8toCalib(Eigen::Matrix4d::Identity());
  Eigen::Matrix4d T_8toCalib_inv(Eigen::Matrix4d::Identity());
  T_8toCalib.block<3,3>(0,0) << -0.707107, 0.0, 0.707107,
                                -0.707107, 0.0, -0.707107,
                                 0.0, -1.0, 0.0;
  T_8toCalib.block<3,1>(0,3) << 0.0944, -0.0944, 0.009;

  Eigen::Matrix4d T_tracker2tool(Eigen::Matrix4d::Identity());
  T_tracker2tool.block<3,3>(0,0) <<   -1.0000000,  0.0000000,  0.0000000,
                                       0.0000000,  1.0000000,  0.0000000,
                                      -0.0000000,  0.0000000, -1.0000000;
  T_tracker2tool.block<3,1>(0,3) << 0.0,0.0,-0.10;

  Eigen::Matrix4d T_tool2link7(Eigen::Matrix4d::Identity());
  T_tool2link7.block<3,1>(0,3) << 0.0,0.0,-0.13;

  Eigen::Matrix3d r_tmp(T_8toCalib.block<3,3>(0,0));
  T_8toCalib_inv.block<3,3>(0,0) = r_tmp.transpose();
  T_8toCalib_inv.block<3,1>(0,3) = -r_tmp.transpose() * T_8toCalib.block<3,1>(0,3);
  T_8toCalib_inv.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  T_8toCalib_inv(3,3) = 1.0;

  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformBroadcaster br_base_2_des;

	
	while(ros::ok())
	{
	    if(flag_calibration)
	    {
	      Eigen::Matrix4d tracker_run;

	      tracker_run = Eigen::Matrix4d::Identity();
	      tracker_run.block<3,1>(0,3) << pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z;
	      Eigen::Quaterniond q_tmp;
	      q_tmp.w() = pose_tracker_.orientation.w;
	      q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
	      Eigen::Matrix3d R_tmp(q_tmp);
	      tracker_run.block<3,3>(0,0) = R_tmp;

	      Eigen::Matrix4d T_Calib_to_tracker = tracker_offset_inv_ * tracker_run;

	      // T_0tolink8Result_ = T_0to8_calib_ * T_8toCalib * T_Calib_to_tracker * T_8toCalib_inv;
        Eigen::Matrix4d T_02toolResult_ = T_0to8_calib_ * T_8toCalib * T_Calib_to_tracker * T_tracker2tool;
	      T_0tolink8Result_ = T_02toolResult_ * T_tool2link7;

	      q_tmp = T_Calib_to_tracker.block<3,3>(0,0);
	      tf::Transform transform;
	      tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
	      transform.setRotation(q);
	      transform.setOrigin( tf::Vector3(T_Calib_to_tracker(0,3), T_Calib_to_tracker(1,3), T_Calib_to_tracker(2,3)));
	      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/calib", "/tracker"));

	      q_tmp = T_02toolResult_.block<3,3>(0,0);
	      tf::Transform transform1;
	      tf::Quaternion q1(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
	      transform1.setRotation(q1);
	      transform1.setOrigin( tf::Vector3(T_02toolResult_(0,3), T_02toolResult_(1,3), T_02toolResult_(2,3)));
	      tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/panda_link0", "/toolResult"));

        q_tmp = T_0tolink8Result_.block<3,3>(0,0);
        tf::Transform transform2;
        tf::Quaternion q2(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
        transform2.setRotation(q2);
        transform2.setOrigin( tf::Vector3(T_0tolink8Result_(0,3), T_0tolink8Result_(1,3), T_0tolink8Result_(2,3)));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/panda_link0", "/link8Result"));

	    }

	    switch(state_)
	    {
	      case 0: // idle
	      {
	        break;
	      }
	      case 1: //calibration
	      {
	        Eigen::Matrix4d tracker_offset;

	        tracker_offset = Eigen::Matrix4d::Identity();
	        tracker_offset.block<3,1>(0,3) << pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z;
	        Eigen::Quaterniond q_tmp;
	        q_tmp.w() = pose_tracker_.orientation.w;
	        q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
	        Eigen::Matrix3d R_tmp(q_tmp);
	        tracker_offset.block<3,3>(0,0) = R_tmp;

	        Eigen::Matrix3d r_tmp(tracker_offset.block<3,3>(0,0));
	        tracker_offset_inv_.block<3,3>(0,0) = r_tmp.transpose();
	        tracker_offset_inv_.block<3,1>(0,3) = -r_tmp.transpose() * tracker_offset.block<3,1>(0,3);
	        tracker_offset_inv_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
	        tracker_offset_inv_(3,3) = 1.0;


	        tf::TransformListener tf_listener_;
	        tf::StampedTransform transform_tmp;
	        bool tf_flag = true;
	        while(tf_flag)
	        {
	          try{
	              tf_listener_.lookupTransform("/panda_link0", "/panda_link8", ros::Time(0), transform_tmp);
	              T_0to8_calib_ = Eigen::Matrix4d::Identity();
	              T_0to8_calib_.block<3,1>(0,3) << transform_tmp.getOrigin().x(), transform_tmp.getOrigin().y(), transform_tmp.getOrigin().z();
	              Eigen::Quaterniond q_tmp;
	              q_tmp.w() = transform_tmp.getRotation().w();
	              q_tmp.vec() << transform_tmp.getRotation().x(), transform_tmp.getRotation().y(), transform_tmp.getRotation().z();
	              Eigen::Matrix3d R_tmp(q_tmp);
	              T_0to8_calib_.block<3,3>(0,0) = R_tmp;
	              // std::cout << transform_tmp.getOrigin().x() << " " << transform_tmp.getOrigin().y() << " " << transform_tmp.getOrigin().z() << std::endl;
	              tf_flag = false;
	              flag_calibration = true;
	          }
	          catch (tf::TransformException ex){
	              ROS_ERROR("%s",ex.what());
	              ros::Duration(1.0).sleep();
	          }
	        }
	        state_ = 0;
	        break;
	      }

	      case 2: //save point
	      {
	        geometry_msgs::Pose pose_tmp;
	        pose_tmp.position.x = T_0tolink8Result_(0,3);
	        pose_tmp.position.y = T_0tolink8Result_(1,3);
	        pose_tmp.position.z = T_0tolink8Result_(2,3);
	        Eigen::Quaterniond q_tmp(T_0tolink8Result_.block<3,3>(0,0));
	        pose_tmp.orientation.w = q_tmp.w();
	        pose_tmp.orientation.x = q_tmp.x();
	        pose_tmp.orientation.y = q_tmp.y();
	        pose_tmp.orientation.z = q_tmp.z();
	        vec_point_.push_back(pose_tmp);
	        vec_task_type_.push_back(0);
	        state_ = 0;
	        break;
	      }

	      case 3: // start traj
	      {
	        geometry_msgs::Pose pose_tmp;
	        pose_tmp.position.x = T_0tolink8Result_(0,3);
	        pose_tmp.position.y = T_0tolink8Result_(1,3);
	        pose_tmp.position.z = T_0tolink8Result_(2,3);
	        Eigen::Quaterniond q_tmp(T_0tolink8Result_.block<3,3>(0,0));
	        pose_tmp.orientation.w = q_tmp.w();
	        pose_tmp.orientation.x = q_tmp.x();
	        pose_tmp.orientation.y = q_tmp.y();
	        pose_tmp.orientation.z = q_tmp.z();
          pose_array_tmp_.poses.push_back(pose_tmp);
	        break;
	      }

	      case 4: // end traj
	      {
          vec_traj_.push_back(pose_array_tmp_);
          pose_array_tmp_.poses.clear();
	        vec_task_type_.push_back(1);
	        state_ = 0;
	        break;
	      }

	      case 5: // clear
	      {
	      	vec_task_type_.clear();
	        vec_traj_.clear();
          pose_array_tmp_.poses.clear();
	        state_ = 0;
	        break;
	      }

	      case 6: // play
	      {
	      	if(vec_task_type_.size() > count_play_task_ )
	      	{
	      		if(vec_task_type_[count_play_task_] == 0)
		      	{
		      		if(fist_time)
		      		{
		      			geometry_msgs::Pose pose_tmp = vec_point_[count_play_point_];
		      			T_point_.block<3,1>(0,3) << pose_tmp.position.x, pose_tmp.position.y, pose_tmp.position.z;
		      			Eigen::Quaterniond q_tmp1;
		      			q_tmp1.w() = pose_tmp.orientation.w;
		      			q_tmp1.vec() << pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z;
		      			Eigen::Matrix3d R_tmp(q_tmp1);
		      			T_point_.block<3,3>(0,0) = R_tmp;
		      			bool tf_flag = true;
		      			tf::TransformListener tf_listener_;
	        			tf::StampedTransform transform_tmp;
				        while(tf_flag)
				        {
				          try{
				              tf_listener_.lookupTransform("/panda_link0", "/panda_link8", ros::Time(0), transform_tmp);
				              T_0to8_ = Eigen::Matrix4d::Identity();
				              T_0to8_.block<3,1>(0,3) << transform_tmp.getOrigin().x(), transform_tmp.getOrigin().y(), transform_tmp.getOrigin().z();
				              Eigen::Quaterniond q_tmp;
				              q_tmp.w() = transform_tmp.getRotation().w();
				              q_tmp.vec() << transform_tmp.getRotation().x(), transform_tmp.getRotation().y(), transform_tmp.getRotation().z();
				              Eigen::Matrix3d R_tmp(q_tmp);
				              T_0to8_.block<3,3>(0,0) = R_tmp;
				              // std::cout << transform_tmp.getOrigin().x() << " " << transform_tmp.getOrigin().y() << " " << transform_tmp.getOrigin().z() << std::endl;
				              tf_flag = false;
  				          }
  				          catch (tf::TransformException ex){
  				              ROS_ERROR("%s",ex.what());
  				              ros::Duration(1.0).sleep();
  				          }
				        }
				        fist_time = false;
		      		}
		      		else
		      		{
		      			if(step_play <= (time_to_point/dt))
    						{
    							double t = dt * step_play;
    							Eigen::Matrix4d result = pose_interp(t, 0.0, time_to_point, T_0to8_, T_point_);
    							step_play++;
    							// publish pose des result
                  geometry_msgs::Pose des_pose;
                  des_pose.position.x = result(0,3);
                  des_pose.position.y = result(1,3);
                  des_pose.position.z = result(2,3);
                  Eigen::Quaterniond q_tmp(result.block<3,3>(0,0));
                  des_pose.orientation.w = q_tmp.w();
                  des_pose.orientation.x = q_tmp.x();
                  des_pose.orientation.y = q_tmp.y();
                  des_pose.orientation.z = q_tmp.z();
                  tf::Transform transform;
                  tf::Quaternion q(des_pose.orientation.x,des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
                  transform.setRotation(q);
                  transform.setOrigin( tf::Vector3(des_pose.position.x, des_pose.position.y, des_pose.position.z));
                  br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/des_pose_link8"));
    						}
    						else
    						{
    							step_play = 0;
    							count_play_task_++;
                  count_play_point_++;
    							fist_time = true;
    						}

		      		}
		      	}
		      	else if(vec_task_type_[count_play_task_] == 1)
		      	{
              if(vec_traj_[count_play_traj_].poses.size() > step_play)
              {
                geometry_msgs::Pose pose_tmp = vec_traj_[count_play_traj_].poses[step_play];
                Eigen::Matrix4d result(Eigen::Matrix4d::Identity());
                result.block<3,1>(0,3) << pose_tmp.position.x, pose_tmp.position.y, pose_tmp.position.z;
                Eigen::Quaterniond q_tmp1;
                q_tmp1.w() = pose_tmp.orientation.w;
                q_tmp1.vec() << pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z;
                Eigen::Matrix3d R_tmp(q_tmp1);
                result.block<3,3>(0,0) = R_tmp;
                step_play++;
                // publish pose des result
                geometry_msgs::Pose des_pose;
                des_pose.position.x = result(0,3);
                des_pose.position.y = result(1,3);
                des_pose.position.z = result(2,3);
                Eigen::Quaterniond q_tmp(result.block<3,3>(0,0));
                des_pose.orientation.w = q_tmp.w();
                des_pose.orientation.x = q_tmp.x();
                des_pose.orientation.y = q_tmp.y();
                des_pose.orientation.z = q_tmp.z();
                tf::Transform transform;
                tf::Quaternion q(des_pose.orientation.x,des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
                transform.setRotation(q);
                transform.setOrigin( tf::Vector3(des_pose.position.x, des_pose.position.y, des_pose.position.z));
                br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/des_pose_link8"));
              }
              else
              {
                step_play = 0;
                count_play_task_++;
                count_play_traj_++;
              }              
		      	}
	      	}
	      	else
	      	{
	      		// go home
            count_play_task_ = 0;
            count_play_point_ = 0;
            count_play_traj_ = 0;
            state_ = 0;
	      	}
	        break;
	      }
	    }
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

