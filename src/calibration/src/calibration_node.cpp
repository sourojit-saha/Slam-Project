#include <Eigen/Dense>
#include<cmath>

// ros
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

// utils
#include "utils.h"


class CalibrationNode {
public:
  CalibrationNode(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh),
    nhp_(nhp)
  {
    ROS_INFO("[Calibration] Starting up!");
    bool got_params = true;
  
    got_params &= nhp.getParam("num_calib_clouds", num_calib_clouds_);
    got_params &= nhp.getParam("voxel_size", voxel_size_);
    got_params &= nhp.getParam("velodyne_height", velodyne_height_);
    got_params &= nhp.getParam("robot_1", robot_1_);
    got_params &= nhp.getParam("robot_2", robot_2_);
    got_params &= nhp.getParam("threshold", threshold_);
    got_params &= nhp.getParam("iterations", iterations_);
    got_params &= nhp.getParam("t_dist", t_dist_);
    got_params &= nhp.getParam("t_angle", t_angle_);

    if (!got_params) {
      ROS_FATAL_STREAM("A parameter was not set. Exiting.");
    }
    // Subscribers
    subVelo1 = nh.subscribe<sensor_msgs::PointCloud2> (robot_1_, 5, &CalibrationNode::cloud1CB, this);
    subVelo2 = nh.subscribe<sensor_msgs::PointCloud2> (robot_2_, 5, &CalibrationNode::cloud2CB, this);
    subOdom1 = nh.subscribe<nav_msgs::Odometry> ("/cmu_rc3/integrated_to_init", 5, &CalibrationNode::odom1CB, this);
    subOdom2 = nh.subscribe<nav_msgs::Odometry> ("/cmu_rc4/integrated_to_init", 5, &CalibrationNode::odom2CB, this);

    // Publishers
    pubTransform12 = nh.advertise<nav_msgs::Odometry> ("/transform12", 2);
    pubVelo = nh.advertise<sensor_msgs::PointCloud2> ("/cmu_rc2/velodyne_calibrated", 2);

    ROS_INFO("[Calibration] Started up!");
  }

  void run() {
    // variables
    local_clouds1_.resize(num_calib_clouds_, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
    avail_c1 = false;
    avail_c2 = false;
    avail_d2 = false;
    run_once = true;
    odom1_recieved = false;
    odom1_recieved = false;
    run_again = false;
    cloud_ind1_ = 0;
    cloud_ind2_ = 0;
    // alignment initialization
    T_rg2_rg1 = Eigen::Affine3d::Identity();
    T_g_r1 = Eigen::Affine3d::Identity();
    T_g_r2 = Eigen::Affine3d::Identity();
    T_I = Eigen::Affine3d::Identity();
    local_clouds1_.resize( num_calib_clouds_, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) );
    local_clouds2_.resize( num_calib_clouds_, pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()) );

    ros::Rate r(5);
    while(ros::ok()) {
      ros::spinOnce();

      if(odom1_recieved && odom2_recieved) {
        run_again = odomDiff();
      }
      if(avail_d2) {
        publishTransformedVelo();
      }
      if(avail_c1 && avail_c2 && run_once) {
        avail_c1 = false;
        avail_c2 = false;
        run_once = false;
        processCalibration();
      }
      if(run_again && !run_once && avail_c1 && avail_c2) {
        run_again = false;
        processCalibration();
      }
      r.sleep();
    }
  }

  void cloud1CB(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
    cloud1 = *pc_msg;
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "map";
    
    pcl::fromROSMsg(*pc_msg, *local_clouds1_[cloud_ind1_%num_calib_clouds_]);
    cloud_ind1_++;
    if(cloud_ind1_ == num_calib_clouds_) {
      cloud_ind1_ = 0;
      avail_c1 = true;
    }
  }

  void cloud2CB(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
    cloud2 = *pc_msg;
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = "map";
    avail_d2 = true;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr key_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *local_clouds2_[cloud_ind2_%num_calib_clouds_]);
    cloud_ind2_++;
    if(cloud_ind2_ == num_calib_clouds_) {
      cloud_ind2_ = 0;
      avail_c2 = true;
    }
  }

  void odom1CB(const nav_msgs::Odometry::ConstPtr& msg) {
    odom1 = *msg;
    odom1_recieved = true;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    vehicle1Yaw = yaw;
  }

  void odom2CB(const nav_msgs::Odometry::ConstPtr& msg) {
    odom2 = *msg;
    odom2_recieved = true;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);
    vehicle2Yaw = yaw;
  }

  void processCalibration() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor2(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < local_clouds1_.size(); i++)
    {
      *cloud_sensor1 += *local_clouds1_[i];
    }
    for (int i = 0; i < local_clouds2_.size(); i++)
    {
      *cloud_sensor2 += *local_clouds2_[i];
    }
    // pcl::transformPointCloud(*cloud_sensor2, *cloud_sensor2, T_r2_r1.matrix());

    ROS_INFO("cloud 1 size %d", int(cloud_sensor1->size()));
    ROS_INFO("cloud 2 size %d", int(cloud_sensor2->size()));

    if (cloud_sensor1->size() < 20 || cloud_sensor2->size() < 20)
    {
      ROS_WARN("Pointclouds too small, not proceeding with GICP");
      return;
    }
    // GICP - do we need to check for convergence?
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(threshold_);
    gicp.setMaximumIterations(iterations_);
    gicp.setInputTarget(cloud_sensor1);
    gicp.setInputSource(cloud_sensor2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("Before align");
    gicp.align(*aligned_source, T_rg2_rg1.matrix().template cast<float>());
    ROS_INFO("Before getFinalTransform");
    T_rg2_rg1.matrix() = gicp.getFinalTransformation().template cast<double>();
    ROS_INFO("After getFinalTransform");

    if (gicp.hasConverged() && T_rg2_rg1(0,3) < 0.6 && gicp.getFitnessScore() < 1.0)
    {
      float fitness = gicp.getFitnessScore();
      ROS_WARN("GICP Fitness: %f", fitness);
      ROS_INFO_STREAM(T_rg2_rg1.matrix());

      nav_msgs::Odometry transform_msg;
      transform_msg.header.frame_id = "map";
      transform_msg.child_frame_id = "map2";
      transform_msg.pose.pose = utils::eigenToRosPose(T_rg2_rg1);
      pubTransform12.publish(transform_msg);
      
    }
    else
    {
      ROS_WARN("Robot Autocalibration GICP failed to converge. Not publishing anything. ");
      run_once = true;
    }
  }

  void publishTransformedVelo() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_calib2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud2, *cloud2ptr);
    pcl::transformPointCloud(*cloud2ptr, *cloud_calib2, T_rg2_rg1.matrix());
    pubVelo.publish(*cloud_calib2);
  }

  bool odomDiff() {
    geometry_msgs::Pose odom1_pose = odom1.pose.pose;
    geometry_msgs::Pose odom2_pose = odom2.pose.pose;
    double dist = getDist((odom1_pose.position.x - odom2_pose.position.x), (odom1_pose.position.y - odom2_pose.position.y));
    double angle = getAngle(vehicle1Yaw, vehicle2Yaw);
    T_g_r1 = utils::rosPoseToEigen(odom1_pose);
    T_g_r2 = utils::rosPoseToEigen(odom2_pose);
    T_r2_r1 = T_g_r2.inverse() * T_g_r1 * T_rg2_rg1;
    if(dist < t_dist_ && angle < t_angle_) {
      ROS_WARN("Dist and angle = %f, %f", dist, angle);
    }
    return (dist < t_dist_ && angle < t_angle_);
  }

  double getDist(double x, double y) {
    return sqrt(pow(x,2) + pow(y,2));
  }

  double getAngle(double x, double y) {
    if (abs(x - y) > 2*M_PI) {
      return abs(x - y) - 2*M_PI;
    }
    return abs(x - y);
  }


private:
  ros::Subscriber subVelo1;
  ros::Subscriber subVelo2;
  ros::Subscriber subOdom1;
  ros::Subscriber subOdom2;

  ros::Publisher pubTransform12;
  ros::Publisher pubVelo;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  // params
  int num_calib_clouds_;
  double voxel_size_;
  double velodyne_height_;
  std::string robot_1_;
  std::string robot_2_;
  double threshold_;
  int iterations_;
  double t_dist_;
  double t_angle_;

  size_t cloud_ind1_;
  size_t cloud_ind2_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_clouds1_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_clouds2_;

  bool avail_c1;
  bool avail_c2;
  bool avail_d2;
  bool run_once;
  bool odom1_recieved;
  bool odom2_recieved;
  bool run_again;

  double vehicle1Yaw;
  double vehicle2Yaw;

  sensor_msgs::PointCloud2 cloud1;
  sensor_msgs::PointCloud2 cloud2;
  nav_msgs::Odometry odom1;
  nav_msgs::Odometry odom2;
  geometry_msgs::Pose odom_diff;

  Eigen::Affine3d T_rg2_rg1;
  Eigen::Affine3d T_I;
  Eigen::Affine3d T_g_r1;
  Eigen::Affine3d T_g_r2;
  Eigen::Affine3d T_r2_r1;
}; /* class CalibrationNode */

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  auto node = std::make_shared<CalibrationNode>(nh, nhPrivate);
  node->run();

  return 0;
}