#include "ros/ros.h"
#include "std_msgs/String.h"
#include "franka_msgs/FrankaState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Eigen>
#include <ros_myo/EmgArray.h>

Eigen::VectorXd emg_Arm_;
bool flag_egm_ = false;

void callback_emg_Arm_R(const ros_myo::EmgArray::ConstPtr& msg)
{
    for(int i = 0; i <= 7; i++)
    {
      emg_Arm_(i)= msg->data[i];
      // std::cout<< emg_(0)<<" "<<emg_(1)<<" "<<emg_(2)<<" "<<emg_(3)<<" "<<emg_(4)<<" "<<emg_(5)<<" "<<emg_(6)<<" "<<emg_(7)<<std::endl;

    }
    flag_egm_ = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_stiffness_node");
  ros::NodeHandle n;

  emg_Arm_ = Eigen::VectorXd::Zero(8);

  ros::Publisher pub_stiffness = n.advertise<std_msgs::Float64>("/right_arm/stiffness", 1);

  ros::Subscriber sub_emg_Arm_R = n.subscribe("/For_r_emg", 1, callback_emg_Arm_R);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    if(flag_egm_)
    {
      std_msgs::Float64 Stiff;
      Stiff.data = (emg_Arm_.sum() / 8.0) * 2.0; /// 500.0;
      if(Stiff.data>800.0)Stiff.data = 800.0;
      // if(Stiff.data>700.0)Stiff.data = 700.0;
      else
      {
        if(Stiff.data<200.0)Stiff.data = 200.0;
      }
      pub_stiffness.publish(Stiff);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}