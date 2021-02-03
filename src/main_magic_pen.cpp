
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
Eigen::Matrix4d tracker_offset_inv_, T_0to7_, T_0toResult_;
bool flag_calibration = false;
std::vector<geometry_msgs::Pose>  vec_point_;
std::vector<geometry_msgs::PoseArray>  vec_traj_;
std::vector<int>  vec_task_type_;
int state_ = 0;
int count_traj_ = 0;

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

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "magic_pen_node");
	ros::NodeHandle nh;

  double rate_50Hz = 50.0;
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

  T_0toResult_= Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_7toCalib(Eigen::Matrix4d::Identity());
  Eigen::Matrix4d T_7toCalib_inv(Eigen::Matrix4d::Identity());
  T_7toCalib.block<3,3>(0,0) << -0.707107, 0.0, 0.707107,
                                -0.707107, 0.0, -0.707107,
                                 0.0, -1.0, 0.0;
  T_7toCalib.block<3,1>(0,3) << 0.0944, -0.0944, 0.009;

  Eigen::Matrix4d T_offsetResult(Eigen::Matrix4d::Identity());
  T_offsetResult.block<3,3>(0,0) <<   -1.0000000,  0.0000000,  0.0000000,
                                       0.0000000,  1.0000000,  0.0000000,
                                      -0.0000000,  0.0000000, -1.0000000;

  Eigen::Matrix3d r_tmp(T_7toCalib.block<3,3>(0,0));
  T_7toCalib_inv.block<3,3>(0,0) = r_tmp.transpose();
  T_7toCalib_inv.block<3,1>(0,3) = -r_tmp.transpose() * T_7toCalib.block<3,1>(0,3);
  T_7toCalib_inv.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  T_7toCalib_inv(3,3) = 1.0;

  tf::TransformBroadcaster tf_broadcaster;
	
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

      Eigen::Matrix4d T_Calib_to_Run = tracker_offset_inv_ * tracker_run;

      // T_0toResult_ = T_0to7_ * T_7toCalib * T_Calib_to_Run * T_7toCalib_inv;
      T_0toResult_ = T_0to7_ * T_7toCalib * T_Calib_to_Run * T_offsetResult;

      q_tmp = T_Calib_to_Run.block<3,3>(0,0);
      tf::Transform transform;
      tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
      transform.setRotation(q);
      transform.setOrigin( tf::Vector3(T_Calib_to_Run(0,3), T_Calib_to_Run(1,3), T_Calib_to_Run(2,3)));
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/calib", "/tracker"));

      q_tmp = T_0toResult_.block<3,3>(0,0);
      tf::Transform transform1;
      tf::Quaternion q1(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
      transform1.setRotation(q1);
      transform1.setOrigin( tf::Vector3(T_0toResult_(0,3), T_0toResult_(1,3), T_0toResult_(2,3)));
      tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/panda_link0", "/result"));

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
              T_0to7_ = Eigen::Matrix4d::Identity();
              T_0to7_.block<3,1>(0,3) << transform_tmp.getOrigin().x(), transform_tmp.getOrigin().y(), transform_tmp.getOrigin().z();
              Eigen::Quaterniond q_tmp;
              q_tmp.w() = transform_tmp.getRotation().w();
              q_tmp.vec() << transform_tmp.getRotation().x(), transform_tmp.getRotation().y(), transform_tmp.getRotation().z();
              Eigen::Matrix3d R_tmp(q_tmp);
              T_0to7_.block<3,3>(0,0) = R_tmp;
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
        pose_tmp.position.x = T_0toResult_(0,3);
        pose_tmp.position.y = T_0toResult_(1,3);
        pose_tmp.position.z = T_0toResult_(2,3);
        Eigen::Quaterniond q_tmp(T_0toResult_.block<3,3>(0,0));
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
        pose_tmp.position.x = T_0toResult_(0,3);
        pose_tmp.position.y = T_0toResult_(1,3);
        pose_tmp.position.z = T_0toResult_(2,3);
        Eigen::Quaterniond q_tmp(T_0toResult_.block<3,3>(0,0));
        pose_tmp.orientation.w = q_tmp.w();
        pose_tmp.orientation.x = q_tmp.x();
        pose_tmp.orientation.y = q_tmp.y();
        pose_tmp.orientation.z = q_tmp.z();
        vec_traj_[count_traj_].poses.push_back(pose_tmp);
        break;
      }

      case 4: // end traj
      {
        vec_task_type_.push_back(1);
        count_traj_++;
        state_ = 0;
        break;
      }

      case 5: // clear
      {
        vec_task_type_.clear();
        vec_traj_.clear();
        count_traj_= 0;
        state_ = 0;
        break;
      }
    }
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

