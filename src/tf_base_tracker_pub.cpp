#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::Pose pose_tracker_;
bool flag_callback = false;

void callback_tracker(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  pose_tracker_ = msg->pose.pose;
  flag_callback = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_base_tracker_pub");


  ros::NodeHandle node;
  ros::Rate r_30HZ(30);

  std::vector<double> translation, rotation;
  std::string frame_cam, tracker_base_frame, tracker_frame;
  node.getParam("/translation_tracker", translation);
  node.getParam("/rotation_tracker", rotation);


  frame_cam = "camera1_color_optical_frame";
  tracker_base_frame = "gianluca_vive_world";
  tracker_frame = "gianluca_tracker";

  Eigen::Vector3d vec_cam_2_basetracker;
  Eigen::Quaterniond quat_cam_2_basetracker;
  vec_cam_2_basetracker << translation[0], translation[1], translation[2];
  quat_cam_2_basetracker.w() = rotation[3];
  quat_cam_2_basetracker.vec() << rotation[0], rotation[1], rotation[2];
  Eigen::Matrix4d T_cam_2_basetracker_(Eigen::Matrix4d::Identity());
  Eigen::Matrix3d temp_R_cam_2_basetracker(quat_cam_2_basetracker);
  T_cam_2_basetracker_.block<3,3>(0,0) = temp_R_cam_2_basetracker;
  T_cam_2_basetracker_.block<3,1>(0,3) = vec_cam_2_basetracker;


  static tf::TransformBroadcaster br_cam_2_basetracker;
  tf::Transform transform_cam_2_basetracker;
  transform_cam_2_basetracker.setOrigin( tf::Vector3(T_cam_2_basetracker_(0,3), T_cam_2_basetracker_(1,3), T_cam_2_basetracker_(2,3)));
  tf::Quaternion q_cam_2_base(quat_cam_2_basetracker.x(), quat_cam_2_basetracker.y(), quat_cam_2_basetracker.z(), quat_cam_2_basetracker.w());
  transform_cam_2_basetracker.setRotation(q_cam_2_base);

  static tf::TransformBroadcaster br_basetracker_2_tracker;
  tf::Transform transform_basetracker_2_tracker;
  

  ros::Subscriber sub_tracker_pose = node.subscribe("/vive/LHR_E4F7FAAE_pose", 1, &callback_tracker);

  while(ros::ok())
  {

    br_cam_2_basetracker.sendTransform(tf::StampedTransform(transform_cam_2_basetracker, ros::Time::now(), frame_cam, tracker_base_frame));
    if(flag_callback)
      {
        transform_basetracker_2_tracker.setOrigin( tf::Vector3(pose_tracker_.position.x, pose_tracker_.position.y, pose_tracker_.position.z));
        tf::Quaternion q_basetracker_2_tracker(pose_tracker_.orientation.x, pose_tracker_.orientation.y, pose_tracker_.orientation.z, pose_tracker_.orientation.w);
        transform_basetracker_2_tracker.setRotation(q_basetracker_2_tracker);
        br_basetracker_2_tracker.sendTransform(tf::StampedTransform(transform_basetracker_2_tracker, ros::Time::now(), tracker_base_frame, tracker_frame));
      }
    ros::spinOnce();
    r_30HZ.sleep();
  }


  return 0;
};