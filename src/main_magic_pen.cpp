
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <string>

geometry_msgs::Pose pose_tracker_;
Eigen::Matrix4d tracker_offset_;

void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	pose_tracker_ = msg->pose.pose;
}

bool callback_calibration(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  tracker_offset_ = Eigen::Matrix4d::Identity();
  tracker_offset_.block<3,1>(0,3) << pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z;
  Eigen::Quaterniond q_tmp;
  q_tmp.w() = pose_tracker_.orientation.w;
  q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
  Eigen::Matrix3d R_tmp(q_tmp);
  tracker_offset_.block<3,3>(0,0) = R_tmp;
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
	//Subscriber
	ros::Subscriber sub_tracker_pose = nh.subscribe("/vive/LHR_E4F7FAAE_pose", 1, &callback_tracker);
  //Service Server
  ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

