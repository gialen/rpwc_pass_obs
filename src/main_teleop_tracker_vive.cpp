
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <fstream>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

Eigen::Matrix4d T_base_2_EE_, T_tracker_;
bool flag_T_base_2_EE_ = false;
bool first_quat_tracker_ = true;
bool first_cmd_ = true;
Eigen::Quaterniond quat_tracker_old_;
int state_ = 0;


void callback_curr_pose_EE(const geometry_msgs::Pose::ConstPtr& msg)
{
  Eigen::Quaterniond quat_EE;
  quat_EE.w() = msg->orientation.w;
  quat_EE.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;

  T_base_2_EE_ = Eigen::Matrix4d::Identity();
  T_base_2_EE_(0,3) = msg->position.x;
  T_base_2_EE_(1,3) = msg->position.y;
  T_base_2_EE_(2,3) = msg->position.z;
  Eigen::Matrix3d R_tmp(quat_EE);
  T_base_2_EE_.block<3,3>(0,0) = R_tmp;
  flag_T_base_2_EE_ = true;
}

void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // pose_tracker_ = msg->pose.pose;

  Eigen::Quaterniond quat_tracker;
  quat_tracker.w() = msg->pose.pose.orientation.w;
  quat_tracker.vec() << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;

  // rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
  if(first_quat_tracker_)
  {
    first_quat_tracker_= false;
    quat_tracker_old_ = quat_tracker;
  } 

  double sign_check = quat_tracker.w() * quat_tracker_old_.w() + quat_tracker.x() * quat_tracker_old_.x() + quat_tracker.y() * quat_tracker_old_.y() + quat_tracker.z() * quat_tracker_old_.z();
  if(sign_check < 0.0)
  {
    quat_tracker.w() = quat_tracker.w() * (-1); 
    quat_tracker.vec() = quat_tracker.vec() * (-1); 
  }
  quat_tracker_old_ = quat_tracker;

  T_tracker_ = Eigen::Matrix4d::Identity();
  T_tracker_.block<3,3>(0,0) = quat_tracker.toRotationMatrix();
  T_tracker_.block<3,1>(0,3) << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
}

bool callback_calibration(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 1;
  return true;
}

bool callback_teleop_send_cmd(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
  if(req.data)
  {
    first_cmd_ = true;
    state_ = 2;
  }
  else state_ = 0;
  return true;
}
//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_tracker_vive_node");
	ros::NodeHandle nh;

  double rate_50Hz = 50.0;
  double dt = 1.0/rate_50Hz;
	ros::Rate r_50HZ(rate_50Hz);

  ros::Subscriber sub_robot_pose = nh.subscribe("/setup1/rpwc_robot_curr_pose", 1, callback_curr_pose_EE);
  ros::Subscriber sub_tracker_pose = nh.subscribe("/vive/LHR_042ED232_pose", 1, &callback_tracker);

  ros::Publisher pub_pos_des = nh.advertise<geometry_msgs::Pose>("/setup1/rpwc_pose_des", 1);


  ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);
  ros::ServiceServer server_teleop_send_cmd = nh.advertiseService("/teleop_send_cmd", &callback_teleop_send_cmd);

  bool flag_calibration = false;
  Eigen::Matrix4d T_EEtoCalib(Eigen::Matrix4d::Identity());
  T_EEtoCalib.block<3,3>(0,0) << -0.707107, 0.0, 0.707107,
                                -0.707107, 0.0, -0.707107,
                                 0.0, -1.0, 0.0;
  T_EEtoCalib.block<3,1>(0,3) << 0.0944, -0.0944, 0.009;
  Eigen::Matrix4d tracker_offset_inv_, T_0toEE_calib_, T_base2tracker_, T_base2Result_, T_Result2EE;

  Eigen::Matrix4d T_tracker2result(Eigen::Matrix4d::Identity());
  T_tracker2result.block<3,3>(0,0) <<   1.0000000,  0.0000000,  0.0000000,
                                       0.0000000,  -1.0000000,  0.0000000,
                                      0.0000000,  0.0000000, -1.0000000;

  Eigen::Vector3d offset;

	tf::TransformBroadcaster tf_broadcaster;
	while(ros::ok())
	{
    if(flag_calibration)
    {
      Eigen::Matrix4d T_Calib_to_tracker = tracker_offset_inv_ * T_tracker_;

      // T_Calib_to_tracker.block<3,1>(0,3) << T_Calib_to_tracker(0,3) + offset(0),T_Calib_to_tracker(1,3) + offset(1), T_Calib_to_tracker(2,3) + offset(2);
      T_base2tracker_ = T_0toEE_calib_ * T_EEtoCalib * T_Calib_to_tracker;
      T_base2Result_ = T_base2tracker_ * T_tracker2result;

      Eigen::Quaterniond q_tmp(T_base2Result_.block<3,3>(0,0));
      tf::Transform transform;
      tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
      transform.setRotation(q);
      transform.setOrigin( tf::Vector3(T_base2Result_(0,3), T_base2Result_(1,3), T_base2Result_(2,3)));
      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/tracker"));
    }

		switch(state_)
      {
        case 0: // idle
        {
          break;
        }

        case 1: //calibration
        {

          // T_8toCalib =  handEye(AA, BB);
          // std::cout<< "T_8toCalib: "<< T_8toCalib<< std::endl;

          Eigen::Matrix4d tracker_offset;

          tracker_offset = Eigen::Matrix4d::Identity();
          tracker_offset = T_tracker_;

          Eigen::Matrix3d r_tmp(tracker_offset.block<3,3>(0,0));
          tracker_offset_inv_ = Eigen::Matrix4d::Identity();
          tracker_offset_inv_.block<3,3>(0,0) = r_tmp.transpose();
          tracker_offset_inv_.block<3,1>(0,3) = -r_tmp.transpose() * tracker_offset.block<3,1>(0,3);

          T_0toEE_calib_ = Eigen::Matrix4d::Identity();
          T_0toEE_calib_ = T_base_2_EE_;

          flag_calibration = true;
          state_ = 0;
          std::cout<<"CALIBRATED"<<std::endl;
          break;
        }

        case 2: //teleop
        {
          if(flag_T_base_2_EE_  && flag_calibration)
          {
            if(first_cmd_)
            {
              Eigen::Matrix4d T_base2Result_inv;
              T_base2Result_inv = Eigen::Matrix4d::Identity();
              T_Result2EE = Eigen::Matrix4d::Identity();

              Eigen::Matrix3d r_tmp(T_base2Result_.block<3,3>(0,0));
              T_base2Result_inv.block<3,3>(0,0) = r_tmp.transpose();
              T_base2Result_inv.block<3,1>(0,3) = -r_tmp.transpose() * T_base2Result_.block<3,1>(0,3);

              T_Result2EE = T_base2Result_inv * T_base_2_EE_;
              // offset espresso in result frame
              offset << T_Result2EE(0,3), T_Result2EE(1,3), T_Result2EE(2,3);
              // per sommarla come fatto sotto bisogna riportare l'offset in robot base frame
              offset = T_base2Result_.block<3,3>(0,0) * offset;
              std::cout<<"offset: "<< offset <<std::endl;
              first_cmd_ = false;
            }
            else
            {
              Eigen::Matrix4d T_base2ResultOffset(Eigen::Matrix4d::Identity());
              // T_base2ResultOffset = T_base2Result_ * T_Result2EE;
              T_base2ResultOffset.block<3,3>(0,0) = T_base2Result_.block<3,3>(0,0);
              T_base2ResultOffset.block<3,1>(0,3) << T_base2Result_(0,3) + offset(0),T_base2Result_(1,3) + offset(1), T_base2Result_(2,3) + offset(2);

              Eigen::Quaterniond q_tmp(T_base2ResultOffset.block<3,3>(0,0));
              tf::Transform transform;
              tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
              transform.setRotation(q);
              transform.setOrigin( tf::Vector3(T_base2ResultOffset(0,3), T_base2ResultOffset(1,3), T_base2ResultOffset(2,3)));
              tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/tracker_pose_des"));

              geometry_msgs::Pose msg_des_pose;
              msg_des_pose.position.x = T_base2ResultOffset(0,3);
              msg_des_pose.position.y = T_base2ResultOffset(1,3);
              msg_des_pose.position.z = T_base2ResultOffset(2,3);

              msg_des_pose.orientation.w = q_tmp.w();
              msg_des_pose.orientation.x = q_tmp.x();
              msg_des_pose.orientation.y = q_tmp.y();
              msg_des_pose.orientation.z = q_tmp.z();

              pub_pos_des.publish(msg_des_pose);
            }
          }
          else std::cout<<"teleop error"<<std::endl;



          break;
        }



        // case 7: // homing
        // {
        //   if(fist_time)
        //   {

        //     T_point_.block<3,1>(0,3) << 0.379, -0.006, 0.581;
        //     Eigen::Quaterniond q_tmp1;
        //     q_tmp1.w() = -0.240;
        //     q_tmp1.vec() << 0.756, -0.354, 0.495;
        //     Eigen::Matrix3d R_tmp(q_tmp1);
        //     T_point_.block<3,3>(0,0) = R_tmp;
        //     bool tf_flag = true;
        //     tf::TransformListener tf_listener_;
        //     tf::StampedTransform transform_tmp;
        //     while(tf_flag)
        //     {
        //       try{
        //           tf_listener_.lookupTransform("/panda_link0", "/panda_link8", ros::Time(0), transform_tmp);
        //           T_0to8_ = Eigen::Matrix4d::Identity();
        //           T_0to8_.block<3,1>(0,3) << transform_tmp.getOrigin().x(), transform_tmp.getOrigin().y(), transform_tmp.getOrigin().z();
        //           Eigen::Quaterniond q_tmp;
        //           q_tmp.w() = transform_tmp.getRotation().w();
        //           q_tmp.vec() << transform_tmp.getRotation().x(), transform_tmp.getRotation().y(), transform_tmp.getRotation().z();
        //           Eigen::Matrix3d R_tmp(q_tmp);
        //           T_0to8_.block<3,3>(0,0) = R_tmp;
        //           // std::cout << transform_tmp.getOrigin().x() << " " << transform_tmp.getOrigin().y() << " " << transform_tmp.getOrigin().z() << std::endl;
        //           tf_flag = false;
        //         }
        //         catch (tf::TransformException ex){
        //             ROS_ERROR("%s",ex.what());
        //             ros::Duration(1.0).sleep();
        //         }
        //     }
        //     fist_time = false;
        //   }
        //   else
        //   {
        //     if(step_play <= (time_to_point/dt))
        //     {
        //       double t = dt * step_play;
        //       Eigen::Matrix4d result = pose_interp(t, 0.0, time_to_point, T_0to8_, T_point_);
        //       step_play++;
        //       // publish pose des result
        //       geometry_msgs::Pose des_pose;
        //       des_pose.position.x = result(0,3);
        //       des_pose.position.y = result(1,3);
        //       des_pose.position.z = result(2,3);
        //       Eigen::Quaterniond q_tmp(result.block<3,3>(0,0));
        //       des_pose.orientation.w = q_tmp.w();
        //       des_pose.orientation.x = q_tmp.x();
        //       des_pose.orientation.y = q_tmp.y();
        //       des_pose.orientation.z = q_tmp.z();

        //       geometry_msgs::PoseStamped send_pose;
        //       send_pose.pose = des_pose;
        //       send_pose.header.stamp = ros::Time::now();
        //       pub_pos_des.publish(send_pose);

        //       tf::Transform transform;
        //       tf::Quaternion q(des_pose.orientation.x,des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
        //       transform.setRotation(q);
        //       transform.setOrigin( tf::Vector3(des_pose.position.x, des_pose.position.y, des_pose.position.z));
        //       br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/des_pose_link8"));
        //     }
        //     else
        //     {
        //       step_play = 0;
        //       fist_time = true;
        //       state_ = 0;
        //       std::cout<<"END TASK"<<std::endl;
        //     }

        //   }
          
        //   break;
        // }
      }

		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

