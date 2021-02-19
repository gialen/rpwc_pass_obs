
#include <ros/ros.h>
#include <ros/rate.h>
#include <ros/package.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <fstream>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>


Eigen::Matrix4d T_base_2_EE_, T_joy_;
bool flag_T_base_2_EE_ = false;
bool first_quat_joy_ = true;
bool first_cmd_ = true;
bool flag_send_cmd_ = false;
bool flag_calibration_ = false;
Eigen::Quaterniond quat_joy_old_;
int state_ = 0;
int menu_butt_old_ = 0;
double gripper_ = 0.0;


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

void callback_joy_R_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  Eigen::Quaterniond quat_joy;
  quat_joy.w() = msg->pose.orientation.w;
  quat_joy.vec() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

  // rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
  if(first_quat_joy_)
  {
    first_quat_joy_= false;
    quat_joy_old_ = quat_joy;
  } 

  double sign_check = quat_joy.w() * quat_joy_old_.w() + quat_joy.x() * quat_joy_old_.x() + quat_joy.y() * quat_joy_old_.y() + quat_joy.z() * quat_joy_old_.z();
  if(sign_check < 0.0)
  {
    quat_joy.w() = quat_joy.w() * (-1); 
    quat_joy.vec() = quat_joy.vec() * (-1); 
  }
  quat_joy_old_ = quat_joy;


  T_joy_ = Eigen::Matrix4d::Identity();
  T_joy_(0,3) = msg->pose.position.x;
  T_joy_(1,3) = msg->pose.position.y;
  T_joy_(2,3) = msg->pose.position.z;
  Eigen::Matrix3d R_tmp(quat_joy);
  T_joy_.block<3,3>(0,0) = R_tmp;

}

void callback_joy_R(const sensor_msgs::Joy::ConstPtr& msg)
{
  gripper_ = msg->axes[0];
  int menu_butt = msg->buttons[3];
  if((menu_butt == 1) && (menu_butt_old_ == 0))
  {
    if(!flag_calibration_) state_ = 1;
    else flag_send_cmd_ = !flag_send_cmd_;
  }

   menu_butt_old_ = menu_butt;
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
    flag_send_cmd_ = true;
  }
  else  flag_send_cmd_ = false;
  return true;
}
//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop_joy_vive_node");
	ros::NodeHandle nh;

  double rate_100Hz = 100.0;
  double dt = 1.0/rate_100Hz;
	ros::Rate r_100HZ(rate_100Hz);

  ros::Subscriber sub_robot_pose = nh.subscribe("/setup1/rpwc_robot_curr_pose", 1, callback_curr_pose_EE);
  ros::Subscriber sub_joy_R_pose = nh.subscribe("/right_controller_as_posestamped", 1, &callback_joy_R_pose);
  ros::Subscriber sub_joy_R = nh.subscribe("/vive_right", 1, &callback_joy_R);

  ros::Publisher pub_pos_des = nh.advertise<geometry_msgs::Pose>("/setup1/rpwc_pose_des", 1);
  ros::Publisher pub_CommandHand = nh.advertise<std_msgs::Float64>("/setup1/rpwc_EE_cmd", 1);


  ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);
  ros::ServiceServer server_teleop_send_cmd = nh.advertiseService("/teleop_send_cmd", &callback_teleop_send_cmd);

  
  Eigen::Matrix4d T_joy_calib_inv;
  Eigen::Matrix3d R_baserobot2calibjoy, R_offset_EE;
  R_baserobot2calibjoy = Eigen::Matrix3d::Identity();
  R_baserobot2calibjoy << 0.0,  0.0, -1.0,
                        -1.0,  0.0, -0.0,
                         0.0,  1.0,  0.0;
  R_offset_EE << 1.0,  0.0,  0.0,
                 0.0, -1.0, -0.0,
                 0.0,  0.0, -1.0;

  Eigen::Vector3d pos_EE_offset;
  tf::TransformBroadcaster tf_broadcaster;
  Eigen::Matrix3d r_joy_calib;

	while(ros::ok())
	{

      switch(state_)
      {
        case 0: // idle
        {
          break;
        }

        case 1: //calibration
        {

          Eigen::Matrix4d T_joy_calib;

          T_joy_calib = Eigen::Matrix4d::Identity();
          T_joy_calib = T_joy_;

          r_joy_calib = T_joy_calib.block<3,3>(0,0);
          T_joy_calib_inv = Eigen::Matrix4d::Identity();
          T_joy_calib_inv.block<3,3>(0,0) = r_joy_calib.transpose();
          T_joy_calib_inv.block<3,1>(0,3) = -r_joy_calib.transpose() * T_joy_calib.block<3,1>(0,3);

          pos_EE_offset = T_base_2_EE_.block<3,1>(0,3);


          // T_0toEE_calib_ = Eigen::Matrix4d::Identity();
          // T_0toEE_calib_ = T_base_2_EE_;

          flag_calibration_ = true;
          state_ = 2;
          std::cout<<"CALIBRATED"<<std::endl;
          break;
        }

        case 2: //teleop
        {
          if(flag_T_base_2_EE_  && flag_calibration_)
          {
            Eigen::Matrix4d T_joy_calib2curr;
            T_joy_calib2curr = T_joy_calib_inv * T_joy_;
            Eigen::Matrix3d R_baserobot2result;
            R_baserobot2result = R_baserobot2calibjoy * T_joy_calib2curr.block<3,3>(0,0) * R_baserobot2calibjoy.transpose() * R_offset_EE;
            Eigen::Vector3d V_baserobot2result;
            V_baserobot2result = pos_EE_offset + (R_baserobot2calibjoy * T_joy_calib2curr.block<3,1>(0,3));
            // std::cout << "pos_EE_offset: " << pos_EE_offset<<std::endl;
            // std::cout << "T_joy_calib2curr.block<3,1>(0,3): " << T_joy_calib2curr.block<3,1>(0,3)<<std::endl;

            Eigen::Quaterniond q_tmp(R_baserobot2result);
            tf::Transform transform;
            tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
            transform.setRotation(q);
            transform.setOrigin( tf::Vector3(V_baserobot2result(0), V_baserobot2result(1), V_baserobot2result(2)));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/teleop_pose_des"));

            if(flag_send_cmd_)
            {
              geometry_msgs::Pose msg_des_pose;
              msg_des_pose.position.x = V_baserobot2result(0);
              msg_des_pose.position.y = V_baserobot2result(1);
              msg_des_pose.position.z = V_baserobot2result(2);

              msg_des_pose.orientation.w = q_tmp.w();
              msg_des_pose.orientation.x = q_tmp.x();
              msg_des_pose.orientation.y = q_tmp.y();
              msg_des_pose.orientation.z = q_tmp.z();

              pub_pos_des.publish(msg_des_pose);

              std_msgs::Float64 msg_EE;
              msg_EE.data = gripper_;
              pub_CommandHand.publish(msg_EE);
            }
            else
            {
              Eigen::Matrix4d T_joy_calib;

              T_joy_calib = Eigen::Matrix4d::Identity();
              T_joy_calib = T_joy_;

              // Eigen::Matrix3d r_tmp(T_joy_calib.block<3,3>(0,0));
              // T_joy_calib_inv = Eigen::Matrix4d::Identity();
              // T_joy_calib_inv.block<3,3>(0,0) = r_tmp.transpose();
              //aggiorno solo la parte di posizione nella matrice per la calibrazione 
              T_joy_calib_inv.block<3,1>(0,3) = -r_joy_calib.transpose() * T_joy_calib.block<3,1>(0,3);

              pos_EE_offset = T_base_2_EE_.block<3,1>(0,3);
            }
            // if(first_cmd_)
            // {
            //   Eigen::Matrix4d T_joy_calib;

            //   T_joy_calib = Eigen::Matrix4d::Identity();
            //   T_joy_calib = T_joy_;

            //   // Eigen::Matrix3d r_tmp(T_joy_calib.block<3,3>(0,0));
            //   // T_joy_calib_inv = Eigen::Matrix4d::Identity();
            //   // T_joy_calib_inv.block<3,3>(0,0) = r_tmp.transpose();
            //   //aggiorno solo la parte di posizione nella matrice per la calibrazione 
            //   T_joy_calib_inv.block<3,1>(0,3) = -r_joy_calib.transpose() * T_joy_calib.block<3,1>(0,3);


            //   pos_EE_offset = T_base_2_EE_.block<3,1>(0,3);

            //   first_cmd_ = false;
            // }
            // else
            // {
            //   Eigen::Matrix4d T_joy_calib2curr;
            //   T_joy_calib2curr = T_joy_calib_inv * T_joy_;
            //   Eigen::Matrix3d R_baserobot2result;
            //   R_baserobot2result = R_baserobot2calibjoy * T_joy_calib2curr.block<3,3>(0,0) * R_baserobot2calibjoy.transpose() * R_offset_EE;
            //   Eigen::Vector3d V_baserobot2result;
            //   V_baserobot2result = pos_EE_offset + (R_baserobot2calibjoy * T_joy_calib2curr.block<3,1>(0,3));
            //   // std::cout << "pos_EE_offset: " << pos_EE_offset<<std::endl;
            //   // std::cout << "T_joy_calib2curr.block<3,1>(0,3): " << T_joy_calib2curr.block<3,1>(0,3)<<std::endl;

            //   Eigen::Quaterniond q_tmp(R_baserobot2result);
            //   tf::Transform transform;
            //   tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
            //   transform.setRotation(q);
            //   transform.setOrigin( tf::Vector3(V_baserobot2result(0), V_baserobot2result(1), V_baserobot2result(2)));
            //   tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/teleop_pose_des"));

              

            //   if(flag_send_cmd_)
            //   {
            //     geometry_msgs::Pose msg_des_pose;
            //     msg_des_pose.position.x = V_baserobot2result(0);
            //     msg_des_pose.position.y = V_baserobot2result(1);
            //     msg_des_pose.position.z = V_baserobot2result(2);

            //     msg_des_pose.orientation.w = q_tmp.w();
            //     msg_des_pose.orientation.x = q_tmp.x();
            //     msg_des_pose.orientation.y = q_tmp.y();
            //     msg_des_pose.orientation.z = q_tmp.z();

            //     pub_pos_des.publish(msg_des_pose);
            //   }
            // }
          }
          else std::cout<<"teleop error"<<std::endl;
          break;
        }


      }
    
		ros::spinOnce();
		r_100HZ.sleep();
	}// end while()
	return 0;
}

