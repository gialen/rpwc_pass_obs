
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



Eigen::Matrix4d T_cam2basetrack_, T_cam2marker_, T_marker2track_, T_track2basetrack_;
std::vector<double> kept_translation_x_;
std::vector<double> kept_translation_y_;
std::vector<double> kept_translation_z_;
std::vector<double> kept_rot_axis_x_;
std::vector<double> kept_rot_axis_y_;
std::vector<double> kept_rot_axis_z_;
std::vector<double> kept_rot_angle_;
int samples_;
geometry_msgs::Pose pose_tracker_;

void callback_marker(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	kept_translation_x_.erase(kept_translation_x_.begin());
    kept_translation_y_.erase(kept_translation_y_.begin());
    kept_translation_z_.erase(kept_translation_z_.begin());
    kept_rot_axis_x_.erase(kept_rot_axis_x_.begin());     
    kept_rot_axis_y_.erase(kept_rot_axis_y_.begin());     
    kept_rot_axis_z_.erase(kept_rot_axis_z_.begin());
    kept_rot_angle_.erase(kept_rot_angle_.begin());

    kept_translation_x_.push_back(msg->pose.position.x);
    kept_translation_y_.push_back(msg->pose.position.y);
    kept_translation_z_.push_back(msg->pose.position.z);
    Eigen::AngleAxisd ax ( Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, 
          msg->pose.orientation.y, msg->pose.orientation.z));
    kept_rot_axis_x_.push_back(ax.axis()[0]);     
    kept_rot_axis_y_.push_back(ax.axis()[1]);
    kept_rot_axis_z_.push_back(ax.axis()[2]);
    kept_rot_angle_.push_back(ax.angle());

}

void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	pose_tracker_ = msg->pose.pose;
}

bool callback_calibration(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
	double tx(0),ty(0),tz(0),ax(0),ay(0),az(0),angle(0);
    for (int i =0; i<samples_; ++i)
    {
      tx += kept_translation_x_[i];
      ty += kept_translation_y_[i];
      tz += kept_translation_z_[i];
      ax += kept_rot_axis_x_[i];
      ay += kept_rot_axis_y_[i];
      az += kept_rot_axis_z_[i];
      angle += kept_rot_angle_[i];
    }
    tx /= samples_;
    ty /= samples_;
    tz /= samples_;
    ax /= samples_;
    ay /= samples_;
    az /= samples_;
    angle /= samples_;
            
    Eigen::Vector3d axis (ax,ay,az);
    axis.normalize();
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
    Eigen::Matrix3d R;
    R = q;
    Eigen::Vector3d vec(tx,ty,tz);
    T_cam2marker_.block<3,3>(0,0) = R;
    T_cam2marker_.block<3,1>(0,3) = vec;

    Eigen::Vector3d vec_tmp(pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z);
	Eigen::Quaterniond q_tmp;
	q_tmp.w() = pose_tracker_.orientation.w;
	q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
	Eigen::Matrix3d R_tmp(q_tmp);

	T_track2basetrack_.block<3,3>(0,0) = R_tmp.transpose();
  	T_track2basetrack_.block<3,1>(0,3) = -R_tmp.transpose() * vec_tmp;


    T_cam2basetrack_ = T_cam2marker_ * T_marker2track_ * T_track2basetrack_;
    Eigen::Quaterniond q_to_save;
    q_to_save = T_cam2basetrack_.block<3,3>(0,0);

    std::string path = ros::package::getPath("passive_obs");
    std::string file = path + "/config/" + "cam2basetracker" + ".yaml";
    std::ofstream f;
    f.open(file.c_str());
    if (f.is_open())
    {
      f << "# Recall  x y z qx qy qz qw T_cam2basetrack_" << std::endl;
      f << "translation_tracker: [" << T_cam2basetrack_(0,3) << ", " << 
                          T_cam2basetrack_(1,3) << ", " << 
                          T_cam2basetrack_(2,3)<<"]" << std::endl;
      f << "rotation_tracker: [" << q_to_save.x() << ", " << 
                            q_to_save.y() << ", " << 
                            q_to_save.z() << ", " << 
                            q_to_save.w() << "]" << std::endl;
      f.close();
    }

	return true;
}
//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cam_track_calib_node");
	ros::NodeHandle nh;

  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);
	//Subscriber
	ros::Subscriber sub_marker_pose = nh.subscribe("/aruco_single/pose", 1, &callback_marker);
	ros::Subscriber sub_tracker_pose = nh.subscribe("/vive/LHR_E4F7FAAE_pose", 1, &callback_tracker);
	//Service Server
  	ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);


	samples_ = 50;
    kept_translation_x_.resize(samples_, 0);
    kept_translation_y_.resize(samples_, 0);
    kept_translation_z_.resize(samples_, 0);
    kept_rot_axis_x_.resize(samples_, 0);
    kept_rot_axis_y_.resize(samples_, 0);
    kept_rot_axis_z_.resize(samples_, 0);
    kept_rot_angle_.resize(samples_, 0);

	T_cam2basetrack_ = T_cam2marker_ = T_marker2track_ = T_track2basetrack_ = Eigen::Matrix4d::Identity();
	Eigen::Matrix3d R_marker2track;
	R_marker2track <<  1.0000000,  0.0000000,  0.0000000,
					   0.0000000,  0.0000000, -1.0000000,
					   0.0000000,  1.0000000,  0.0000000;
	T_marker2track_.block<3,3>(0,0) = R_marker2track;

	while(ros::ok())
	{
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

