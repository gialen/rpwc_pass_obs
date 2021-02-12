
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
std::vector<Eigen::Matrix4d> AA; // robot pose
std::vector<Eigen::Matrix4d> BB; // tracker pose
bool first_take_pose_ = true;
Eigen::Matrix4d tracker_offset_inv_calib_(Eigen::Matrix4d::Identity());


void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	pose_tracker_ = msg->pose.pose;
}

bool callback_calibration(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  state_ = 1;
  return true;
}


bool callback_take_pose(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  


  Eigen::Matrix4d tracker_tmp(Eigen::Matrix4d::Identity());
  tracker_tmp.block<3,1>(0,3) << pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z;
  Eigen::Quaterniond q_tmp;
  q_tmp.w() = pose_tracker_.orientation.w;
  q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
  Eigen::Matrix3d R_tmp(q_tmp);
  tracker_tmp.block<3,3>(0,0) = R_tmp;

  if(first_take_pose_)
  {
    first_take_pose_ = false;
    Eigen::Matrix3d r_tmp(tracker_tmp.block<3,3>(0,0));
    tracker_offset_inv_calib_ = Eigen::Matrix4d::Identity();
    tracker_offset_inv_calib_.block<3,3>(0,0) = r_tmp.transpose();
    tracker_offset_inv_calib_.block<3,1>(0,3) = -r_tmp.transpose() * tracker_tmp.block<3,1>(0,3);
  }
  tracker_tmp = tracker_offset_inv_calib_ * tracker_tmp;

  BB.push_back(tracker_tmp);

  Eigen::Matrix4d link8_tmp(Eigen::Matrix4d::Identity());
  tf::TransformListener tf_listener;
  tf::StampedTransform transform_tmp;
  bool tf_flag = true;
  while(tf_flag)
  {
    try{
        tf_listener.lookupTransform("/panda_link0", "/panda_link8", ros::Time(0), transform_tmp);
        link8_tmp.block<3,1>(0,3) << transform_tmp.getOrigin().x(), transform_tmp.getOrigin().y(), transform_tmp.getOrigin().z();
        Eigen::Quaterniond q_tmp;
        q_tmp.w() = transform_tmp.getRotation().w();
        q_tmp.vec() << transform_tmp.getRotation().x(), transform_tmp.getRotation().y(), transform_tmp.getRotation().z();
        Eigen::Matrix3d R_tmp(q_tmp);
        link8_tmp.block<3,3>(0,0) = R_tmp;
        // std::cout << transform_tmp.getOrigin().x() << " " << transform_tmp.getOrigin().y() << " " << transform_tmp.getOrigin().z() << std::endl;
        tf_flag = false;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
  }
  AA.push_back(link8_tmp);
  // state_ = 1;
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
  // result.block<3,1>(0,3) = (1.0 - alpha) * trans1 + alpha * trans2;
  result.block<3,1>(0,3) = trans1 + (((trans2 - trans1) / (t2 - t1)) * t);
  result.block<3,3>(0,0) = rot1.slerp(alpha, rot2).toRotationMatrix();

  return result;
}

Eigen::Matrix3d skew(Eigen::Vector3d V)
{
  Eigen::Matrix3d result;
  result <<  0,    -V(2),    V(1),
        V(2),      0,    -V(0),
       -V(1),    V(0),      0;
  return result;
}

Eigen::Quaterniond rot2quat(Eigen::Matrix4d R)
{
  Eigen::Quaterniond result;
  result.w() = 2 * sqrt(1 + R.block<3,3>(0,0).trace());
  result.vec() << ( R(2,1) - R(1,2) ) / result.w(), ( R(0,2) - R(2,0) ) / result.w(), ( R(1,0) - R(0,1) ) / result.w();
  return result;
}


Eigen::Matrix4d quat2rot(Eigen::Vector3d q)
{
  Eigen::Matrix4d result(Eigen::Matrix4d::Identity());
  double p = q.transpose() * q;
  double w = sqrt(1 - p);
  result.block<3,3>(0,0) = 2*q*q.transpose() + 2*w*skew(q) + Eigen::Matrix3d::Identity() - (2*Eigen::Matrix3d::Identity()*p);

  return result;
}


Eigen::Matrix4d handEye(std::vector<Eigen::Matrix4d> bHg, std::vector<Eigen::Matrix4d> wHc)
{
  int M = bHg.size();
  int K = (M*M-M)/2; //Number of unique camera position pairs

  Eigen::MatrixXd A, B, Pcg_, Pcg, Tcg;
  A = Eigen::MatrixXd::Zero(3*K,3); //will store: skew(Pgij+Pcij)
  B = Eigen::MatrixXd::Zero(3*K,1); //will store: Pcij - Pgij
  Pcg_ = Eigen::MatrixXd::Zero(3,1); //will store: Pcij - Pgij
  Pcg = Eigen::MatrixXd::Zero(3,1); 
  Tcg = Eigen::MatrixXd::Zero(3,1);
  int k = 0;

  // Now convert from wHc notation to Hc notation used in Tsai paper.
  std::vector<Eigen::Matrix4d> Hg = bHg;
  //% Hc = cHw = inv(wHc); We do it in a loop because wHc is given, not cHw
  // std::vector<Eigen::Matrix4d> Hc = zeros(4,4,M);
  std::vector<Eigen::Matrix4d> Hc;
  for(int i = 0; i < M; i++)
  {
    Hc.push_back(wHc[i].inverse());

  }



  for(int i = 0; i < M; i++)
  {
    for(int j = i + 1; j < M; j++)
    {
      Eigen::Matrix4d Hgij = Hg[j].inverse() * Hg[i]; //Transformation from i-th to j-th gripper pose      
      Eigen::Quaterniond Pgij = rot2quat(Hgij); // ... and the corresponding quaternion
      Pgij.w() = Pgij.w() * 2;
      Pgij.vec() = Pgij.vec() * 2;



      Eigen::Matrix4d Hcij = Hc[j] * Hc[i].inverse(); //Transformation from i-th to j-th camera pose
      Eigen::Quaterniond Pcij = rot2quat(Hcij); // ... and the corresponding quaternion
      Pcij.w() = Pcij.w() * 2;
      Pcij.vec() = Pcij.vec() * 2;

      // k = k+1;
      A.block<3,3>(k*3,0) = skew(Pgij.vec()+Pcij.vec()); // left-hand side
      B.block<3,1>(k*3,0) = Pcij.vec() - Pgij.vec(); // right-hand side
      k = k+1;

    }
  }
  
  Pcg_ = A.colPivHouseholderQr().solve(B);                //Solve the equation A*Pcg_ = B


  Eigen::MatrixXd tmp = Pcg_.transpose()*Pcg_;
  Pcg = 2 * Pcg_ / sqrt(1 + tmp(0,0));

  Eigen::Matrix4d Rcg = quat2rot(Pcg/2);         //Rotation matrix

  // Calculate translational component
  k = 0;
  for(int i = 0; i < M; i++)
  {
    for(int j = i + 1; j < M; j++)
    {
      Eigen::Matrix4d Hgij = Hg[j].inverse() * Hg[i]; //Transformation from i-th to j-th gripper pose
      Eigen::Matrix4d Hcij = Hc[j] * Hc[i].inverse(); //Transformation from i-th to j-th camera pose

      A.block<3,3>(k*3,0) = Hgij.block<3,3>(0,0) - Eigen::Matrix3d::Identity(); // left-hand side
      B.block<3,1>(k*3,0) = Rcg.block<3,3>(0,0) * Hcij.block<3,1>(0,3) - Hgij.block<3,1>(0,3); // right-hand side
      k = k+1;
    }
  }

  Tcg = A.colPivHouseholderQr().solve(B);                //Solve the equation A*Pcg_ = B


  Eigen::Matrix4d result(Eigen::Matrix4d::Identity());
  result.block<3,3>(0,0) = Rcg.block<3,3>(0,0);
  result.block<3,1>(0,3) = Tcg;

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
  ros::Publisher pub_pos_des = nh.advertise<geometry_msgs::PoseStamped>("/joint_position_one_task_inv_kin/pose_des", 1);

 //  //Service Server
  ros::ServiceServer server_calibration = nh.advertiseService("/calibrate", &callback_calibration);
  ros::ServiceServer server_take_pose = nh.advertiseService("/take_pose", &callback_take_pose);
  ros::ServiceServer server_save_point = nh.advertiseService("/save_point", &callback_save_point);
  ros::ServiceServer server_start_traj = nh.advertiseService("/start_traj", &callback_start_traj);
  ros::ServiceServer server_end_traj = nh.advertiseService("/end_traj", &callback_end_traj);
  ros::ServiceServer server_clear_task = nh.advertiseService("/clear_task", &callback_clear_task);
  ros::ServiceServer server_play_task = nh.advertiseService("/play_task", &callback_play_task);

  T_0tolink8Result_ = Eigen::Matrix4d::Identity();
  T_point_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_8toCalib(Eigen::Matrix4d::Identity());
  Eigen::Matrix4d T_8toCalib_inv(Eigen::Matrix4d::Identity());
  // T_8toCalib.block<3,3>(0,0) << -0.707107, 0.0, 0.707107,
  //                               -0.707107, 0.0, -0.707107,
  //                                0.0, -1.0, 0.0;
  // T_8toCalib.block<3,1>(0,3) << 0.0944, -0.0944, 0.009;

  Eigen::Matrix4d T_tracker2tool(Eigen::Matrix4d::Identity());
  T_tracker2tool.block<3,3>(0,0) <<   -1.0000000,  0.0000000,  0.0000000,
                                       0.0000000,  1.0000000,  0.0000000,
                                      -0.0000000,  0.0000000, -1.0000000;
  T_tracker2tool.block<3,1>(0,3) << 0.0,0.0,-0.123;

  Eigen::Matrix4d T_tool2link7(Eigen::Matrix4d::Identity());
  T_tool2link7.block<3,1>(0,3) << 0.0,0.0,-0.133;

  Eigen::Matrix3d r_tmp(T_8toCalib.block<3,3>(0,0));
  T_8toCalib_inv.block<3,3>(0,0) = r_tmp.transpose();
  T_8toCalib_inv.block<3,1>(0,3) = -r_tmp.transpose() * T_8toCalib.block<3,1>(0,3);
  T_8toCalib_inv.block<1,3>(3,0) = Eigen::Vector3d::Zero();
  T_8toCalib_inv(3,3) = 1.0;

  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformBroadcaster br_base_2_des;

//   std::vector<Eigen::Matrix4d> test_a, test_b;
//   Eigen::Matrix4d tmp;
//    tmp<<  0.0000,   -1.0000,         0,         0,
//         1.0000,    0.0000,         0,         0,
//              0 ,        0 ,   1.0000 ,        0,
//              0 ,        0  ,       0  ,  1.0000;
//   test_a.push_back(tmp);
//   tmp <<   1.0000 ,        0 ,       0  ,       0,
//          0  ,  0.0000 ,  -1.0000 ,        0,
//          0 ,   1.0000 ,   0.0000 ,        0,
//          0  ,       0 ,        0 ,   1.0000;
//   test_a.push_back(tmp);
//   tmp <<     0.0000 ,        0,    1.0000 ,        0,
//          0,    1.0000,         0 ,        0,
//    -1.0000,         0,    0.0000,         0,
//          0,         0,         0,    1.0000;
//   test_a.push_back(tmp);





//    tmp<<  0.0000 ,  -1.0000,         0,         0,
//     1.0000,    0.0000,         0,         0,
//          0,         0 ,   1.0000 ,        0,
//          0,        0 ,        0 ,   1.0000;
//   test_b.push_back(tmp);
//   tmp <<    0.0000,         0,   -1.0000,   -1.0000,
//          0,    1.0000,         0,         0,
//     1.0000,         0,    0.0000,   -1.0000,
//          0,        0,         0,    1.0000;
//   test_b.push_back(tmp);
//   tmp <<    1.0000,         0,         0,         0,
//          0 ,   0.0000 ,  -1.0000,   -1.0000,
//          0,    1.0000 ,   0.0000,   -1.0000,
//          0  ,       0 ,        0 ,   1.0000;
//   test_b.push_back(tmp);

// Eigen::Matrix4d result =  handEye(test_a, test_b);
// std::cout<< "resutl: "<< result<< std::endl;

	
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
        Eigen::Matrix4d T_02tracker_ = T_0to8_calib_ * T_8toCalib * T_Calib_to_tracker;
        Eigen::Matrix4d T_02toolResult_ = T_0to8_calib_ * T_8toCalib * T_Calib_to_tracker * T_tracker2tool;
	      T_0tolink8Result_ = T_02toolResult_ * T_tool2link7;

	      q_tmp = T_02tracker_.block<3,3>(0,0);
	      tf::Transform transform;
	      tf::Quaternion q(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
	      transform.setRotation(q);
	      transform.setOrigin( tf::Vector3(T_02tracker_(0,3), T_02tracker_(1,3), T_02tracker_(2,3)));
	      tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/tracker"));

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

        q_tmp = T_8toCalib.block<3,3>(0,0);
        tf::Transform transform3;
        tf::Quaternion q3(q_tmp.x(),q_tmp.y(), q_tmp.z(), q_tmp.w());
        transform3.setRotation(q3);
        transform3.setOrigin( tf::Vector3(T_8toCalib(0,3), T_8toCalib(1,3), T_8toCalib(2,3)));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "/panda_link8", "/test_calib"));

	    }

	    switch(state_)
	    {
	      case 0: // idle
	      {
	        break;
	      }
	      case 1: //calibration
	      {

          T_8toCalib =  handEye(AA, BB);
          std::cout<< "T_8toCalib: "<< T_8toCalib<< std::endl;

	        Eigen::Matrix4d tracker_offset;

	        tracker_offset = Eigen::Matrix4d::Identity();
	        tracker_offset.block<3,1>(0,3) << pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z;
	        Eigen::Quaterniond q_tmp;
	        q_tmp.w() = pose_tracker_.orientation.w;
	        q_tmp.vec() << pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z;
	        Eigen::Matrix3d R_tmp(q_tmp);
	        tracker_offset.block<3,3>(0,0) = R_tmp;

	        Eigen::Matrix3d r_tmp(tracker_offset.block<3,3>(0,0));
          tracker_offset_inv_ = Eigen::Matrix4d::Identity();
	        tracker_offset_inv_.block<3,3>(0,0) = r_tmp.transpose();
	        tracker_offset_inv_.block<3,1>(0,3) = -r_tmp.transpose() * tracker_offset.block<3,1>(0,3);



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
          std::cout<<"CALIBRATED"<<std::endl;
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
          std::cout<<"POINT SAVED: "<< T_0tolink8Result_(0,3)<<", "<< T_0tolink8Result_(1,3)<<", "<< T_0tolink8Result_(2,3)<<std::endl;
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
          std::cout<<"TRAJ. SAVED"<<std::endl;
	        break;
	      }

	      case 5: // clear
	      {
	      	vec_task_type_.clear();
          vec_point_.clear();
	        vec_traj_.clear();
          pose_array_tmp_.poses.clear();
          count_play_task_ = 0;
          count_play_point_ = 0;
          count_play_traj_ = 0;
	        state_ = 0;
          step_play = 0;
          std::cout<<"CLEAR"<<std::endl;
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

                std::cout<<"POINT play: "<< T_point_(0,3)<<", "<< T_point_(1,3)<<", "<< T_point_(2,3)<<std::endl;

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

                  geometry_msgs::PoseStamped send_pose;
                  send_pose.pose = des_pose;
                  send_pose.header.stamp = ros::Time::now();
                  pub_pos_des.publish(send_pose);

                  tf::Transform transform;
                  tf::Quaternion q(des_pose.orientation.x,des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
                  transform.setRotation(q);
                  transform.setOrigin( tf::Vector3(des_pose.position.x, des_pose.position.y, des_pose.position.z));
                  br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/des_pose_link8"));
    						
                  Eigen::Quaterniond q_tmp4;
                  q_tmp4 = T_point_.block<3,3>(0,0);
                  tf::Transform transform2;
                  tf::Quaternion q2(q_tmp4.x(),q_tmp4.y(), q_tmp4.z(), q_tmp4.w());
                  transform2.setRotation(q2);
                  transform2.setOrigin( tf::Vector3(T_point_(0,3), T_point_(1,3), T_point_(2,3)));
                  tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/panda_link0", "/point"));

                }
    						else
    						{
    							step_play = 0;
    							count_play_task_++;
                  count_play_point_++;
    							fist_time = true;
                  getchar();
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

                geometry_msgs::PoseStamped send_pose;
                send_pose.pose = des_pose;
                send_pose.header.stamp = ros::Time::now();
                pub_pos_des.publish(send_pose);

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
            state_ = 7;
            std::cout<<"HOMING"<<std::endl;

	      	}
	        break;
	      }

        case 7: // homing
        {
          if(fist_time)
          {

            T_point_.block<3,1>(0,3) << 0.379, -0.006, 0.581;
            Eigen::Quaterniond q_tmp1;
            q_tmp1.w() = -0.240;
            q_tmp1.vec() << 0.756, -0.354, 0.495;
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

              geometry_msgs::PoseStamped send_pose;
              send_pose.pose = des_pose;
              send_pose.header.stamp = ros::Time::now();
              pub_pos_des.publish(send_pose);

              tf::Transform transform;
              tf::Quaternion q(des_pose.orientation.x,des_pose.orientation.y, des_pose.orientation.z, des_pose.orientation.w);
              transform.setRotation(q);
              transform.setOrigin( tf::Vector3(des_pose.position.x, des_pose.position.y, des_pose.position.z));
              br_base_2_des.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/panda_link0", "/des_pose_link8"));
            }
            else
            {
              step_play = 0;
              fist_time = true;
              state_ = 0;
              std::cout<<"END TASK"<<std::endl;
            }

          }
          
          break;
        }
	    }
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

