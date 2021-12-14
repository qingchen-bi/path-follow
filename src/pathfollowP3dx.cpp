#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <utility>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>

//自定义消息
#include "path_follow/ps.h"
//可视化
#include "/home/bqc/catkin_mp_ws/src/path_follow/include/path_follow/visual.h" //TODO 需要修改路径
visualization_msgs::Marker assigned_goal_vis;
path_follow::ps pps;

ros::Subscriber poseSub_;
ros::Publisher cmd_velPub_;
geometry_msgs::Twist vel_des_;
Eigen::Vector3d pose_des_;
Eigen::Vector3d pose_cur_;
std::string wp_flie_path_;
std::vector<std::vector<double> > des_pose_;
// constraints
double theta_err2_, max_angular_vel_, max_linear_vel_, dis_toNextTarget_;
// control gains
double kp_, ki_, kd_;
// flags
bool isToMove_, initTracking_;
// kdTree used for target finding
pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtreeGlobalPath_;
pcl::PointCloud<pcl::PointXY>::Ptr path_pcl_;
pcl::PointXY tmp_node_;
std::vector<int> nearestIndex_;
std::vector<float> nearestDis_;
int tarIndex_;
float tarDis_;

inline double Distance(double x1, double y1, double x2, double y2)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dist = hypot(dx, dy);
  return dist;
}

inline bool isGoalReached(const Eigen::Vector3d& robot_pose)
{
  double distance =
      Distance(robot_pose[0], robot_pose[1], des_pose_[des_pose_.size()-1][0], des_pose_[des_pose_.size()-1][1]);

  if (distance < 0.3)
    return true;
  else
    return false;
}

inline double NormalizeAngle(double angle)
{
  while (angle < -M_PI)
  {
    if (angle < -(2.0 * M_PI))
    {
      angle += (int)(angle / -(2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle += (2.0 * M_PI);
    }
  }

  while (angle > M_PI)
  {
    if (angle > (2.0 * M_PI))
    {
      angle -= (int)(angle / (2.0 * M_PI)) * (2.0 * M_PI);
    }
    else
    {
      angle -= (2.0 * M_PI);
    }
  }

  assert(angle >= -M_PI && angle <= M_PI);

  return angle;
}

inline void getTarget(Eigen::Vector3d& targetPose)
{
  if(des_pose_.empty()){

    ROS_ERROR("no path, so no target to track!!!");
    exit(0);
  }

  // get the nearest waypoint
  tmp_node_.x = (float)pose_cur_[0];
  tmp_node_.y = (float)pose_cur_[1];
  kdtreeGlobalPath_->nearestKSearch(tmp_node_, 1, nearestIndex_, nearestDis_);
  int globalPath_size = des_pose_.size();

  // extend the target forward by dis_toNextTarget_
  tarIndex_ = nearestIndex_[0];
  tarDis_ = nearestDis_[0];
  for(size_t idx = nearestIndex_[0] + 1; idx < globalPath_size; ++idx){
    if(tarDis_ < dis_toNextTarget_)
    {
      tarIndex_ = idx;
      tarDis_ = sqrt( pow((des_pose_[idx][0] - tmp_node_.x), 2) + 
                      pow((des_pose_[idx][1] - tmp_node_.y), 2) );
    } else {
      break;
    }
  }

  targetPose[0] = des_pose_[tarIndex_][0];
  targetPose[1] = des_pose_[tarIndex_][1];
  targetPose[2] = 0.0;
}

inline void controller(const Eigen::Vector3d& targetPose, geometry_msgs::Twist& Twist)
{
  // linear velocity
  double dis_ = Distance(pose_des_[0], pose_des_[1], pose_cur_[0], pose_cur_[1]);
  Twist.linear.x = kp_ * dis_;

  // angular velocity
  double theta = atan2(pose_des_[1] - pose_cur_[1], pose_des_[0] - pose_cur_[0]);
  double theta_err1 = NormalizeAngle(theta - pose_cur_[2]);
  if(initTracking_)
  {
    ROS_INFO("Iniliatizing");
    Twist.linear.x = 0.0;
    Twist.angular.z = 0.5 * theta_err1;
    if (Twist.angular.z > max_angular_vel_)
      Twist.angular.z = max_angular_vel_;
    else if (Twist.angular.z < -max_angular_vel_)
      Twist.angular.z = -max_angular_vel_;

    if(fabs(theta_err1) < 0.174)
      initTracking_ = false;
    return;
  }
  // ROS_WARN("Initialization completed!");
  Twist.angular.z = 0.5 * kp_ * theta_err1 + kd_ * NormalizeAngle(theta_err1 - theta_err2_);

  if (Twist.angular.z > max_angular_vel_)
    Twist.angular.z = max_angular_vel_;
  else if (Twist.angular.z < -max_angular_vel_)
    Twist.angular.z = -max_angular_vel_;

  if(Twist.linear.x > max_linear_vel_)
    Twist.linear.x = max_linear_vel_;
  else if(Twist.linear.x < -max_linear_vel_)
    Twist.linear.x = -max_linear_vel_;
}

void poseCallback(const nav_msgs::Odometry& current_Pose)
{
  pose_cur_[0] = current_Pose.pose.pose.position.x;
  pose_cur_[1] = current_Pose.pose.pose.position.y;
  pose_cur_[2] = tf::getYaw(current_Pose.pose.pose.orientation);
  // std::cout << "current yaw angle: " << pose_cur_[2] << std::endl;
  if(isGoalReached(pose_cur_)) {
    isToMove_ = false;
    cmd_velPub_.shutdown();
    ROS_INFO("Path tracking is finished!");
  }
}

bool txt2Vector(std::vector< std::vector<double> >& res, const std::string& filename , path_follow::ps & visps)
{
  std::fstream input_file;
  input_file.open(filename, std::ios::in);
  assert(input_file.is_open());
  std::string str;
  while (getline(input_file, str))
  {
    std::vector<double> temp;
    // std::istringstream is(str);// 将读出的一行转化为数据流进行操作
    // string->char *
    char *s_input = (char *)str.c_str();
    const char *split = ",";
    // 以‘,’为分隔符拆分字符串
    char *p = strtok(s_input, split);
    double a;
    while (p != NULL)
    {
        // char * -> int
        a = atof(p);
        // cout << a << "|";
        temp.push_back(a);
        p = strtok(NULL, split);
    } //end while

    res.push_back(temp);
  }
  input_file.close();
  if (res.size() == 0) {
    std::cout << res.size() << std::endl;
    return false;
  }
  for(int i = 0; i < res.size(); i++)
  {
        
        
        geometry_msgs::Point pp;
        pp.x = res[i][0];
        pp.y = res[i][1]; 
        visps.points.push_back(pp);
        // assigned_goal_vis = Visual_point(mapData.header.frame_id, 1, 1.0, 0.0, 0.0, visps, 0.4);
        // Assigned_visual_pub.publish(assigned_goal_vis);
  }
  ROS_WARN("waypoints number is : %d", res.size());
  // int i = 0;
  // while (i < res.size()) {
  //   std::cout << res[i][0] << "..." << res[i][1] << std::endl;
  //   i++;
  // }
  return true;
}

bool setParam()
{
  bool ret = true;
  //路径点储存位置 
  wp_flie_path_ = "/home/bqc/catkin_mp_ws/src/path_follow/include/followingpath.txt"; //TODO 需要修改路径
  if (!ros::param::get( "/wp_path", wp_flie_path_)) {
    ROS_WARN("No wp_path found. Looking for %s. Default is txt.",
        "/wp_path");
  }
  max_angular_vel_ = 0.3;
  if (!ros::param::get("/max_angular_vel", max_angular_vel_)) {
    ROS_WARN("No max_angular_vel constraint found. Looking for %s. Default value is 0.5.", "/max_angular_vel");
  }
  max_linear_vel_ = 0.55;
  if (!ros::param::get("/max_linear_vel", max_linear_vel_)) {
    ROS_WARN("No max_linear_vel constraint found. Looking for %s. Default value is 0.3.", "/max_linear_vel");
  }
  kp_ = 1.0;
  if (!ros::param::get( "/kp", kp_)) {
    ROS_WARN("No propotion gain found. Looking for %s. Default value is 1.0.", "/kp");
  }

  ki_ = 1.0;
  if (!ros::param::get("/ki", ki_)) {
    ROS_WARN("No integral gain found. Looking for %s. Default value is 1.0.", "/ki");
  }

  kd_ = 0.8;
  if (!ros::param::get("/kd", kd_)) {
    ROS_WARN("No differential gain found. Looking for %s. Default value is 1.0.", "/kd");
  }

  dis_toNextTarget_ = 0.5;
  if (!ros::param::get("/dis_toNextTarget", dis_toNextTarget_)) {
    ROS_WARN("No dis_toNextTarget found. Looking for %s. Default value is 0.3.", "/dis_toNextTarget");
  }

  return ret;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "p3dx_vel");
  ros::NodeHandle nh;
    ros::Publisher Assigned_visual_pub = nh.advertise<visualization_msgs::Marker>("/Assigned_vis", 10);
  if (!setParam())
  {
    ROS_ERROR("Could not configure the required parammeters!");
  }
  // subcribers
  poseSub_ = nh.subscribe("/RosAria/pose", 10, poseCallback);

  // publishers
  cmd_velPub_  = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);

  if (!txt2Vector(des_pose_, wp_flie_path_, pps)) {
    ROS_ERROR("Could not get the desired pose data!");
  }


  // creat kdTree for target searching
  kdtreeGlobalPath_.reset(new pcl::KdTreeFLANN<pcl::PointXY>());
  path_pcl_.reset(new pcl::PointCloud<pcl::PointXY>);
  nearestIndex_.resize(2);
  nearestDis_.resize(2);
  int i = 0;
  path_pcl_->clear();
  while (i < des_pose_.size()) {
    tmp_node_.x = (float)des_pose_[i][0];
    tmp_node_.y = (float)des_pose_[i][1];
    path_pcl_->push_back(tmp_node_);
    i++;
  }
  kdtreeGlobalPath_->setInputCloud(path_pcl_);

  theta_err2_ = 0.0;
  initTracking_ = true;
  ros::Rate loop_rate_(50);
  while (ros::ok()) {        assigned_goal_vis = Visual_point("map", 1, 1.0, 0.0, 0.0, pps, 0.08);
        Assigned_visual_pub.publish(assigned_goal_vis);
    // PID controller
    // get local target position
    getTarget(pose_des_);

    // calculate desired velocity
    geometry_msgs::Twist cmd_vel;
    controller(pose_des_, cmd_vel);

    // publish velocity
    cmd_velPub_.publish(cmd_vel);

    ros::spinOnce();
    loop_rate_.sleep();
  }

  return 0;
}


