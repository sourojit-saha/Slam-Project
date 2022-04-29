#include "utils.h"

namespace utils {

Eigen::Affine3d rosPoseToEigen(const geometry_msgs::Pose& ros_pose) {
  return  Eigen::Translation3d(
            Eigen::Vector3d(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z) 
          ) 
          *
          Eigen::Quaterniond(ros_pose.orientation.w, ros_pose.orientation.x,
            ros_pose.orientation.y, ros_pose.orientation.z);
}

geometry_msgs::Pose eigenToRosPose(const Eigen::Affine3d& T_aff) {
  Eigen::Vector3d t = T_aff.translation();
  Eigen::Quaterniond q(T_aff.rotation());

  geometry_msgs::Pose ros_pose;
  ros_pose.position.x = t.x();
  ros_pose.position.y = t.y();
  ros_pose.position.z = t.z();
  ros_pose.orientation.x = q.x();
  ros_pose.orientation.y = q.y();
  ros_pose.orientation.z = q.z();
  ros_pose.orientation.w = q.w();

  return ros_pose;
}

Eigen::Affine3d tfToEigen(const tf::Transform& tf) {
  tf::Vector3 t = tf.getOrigin();
  tf::Quaternion q = tf.getRotation();

  return  Eigen::Translation3d(Eigen::Vector3d(t.x(), t.y(), t.z()) ) 
          *Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

tf::Transform eigenToTf(const Eigen::Affine3d& T_aff) {
  Eigen::Vector3d t = T_aff.translation();
  Eigen::Quaterniond q(T_aff.rotation());
  return tf::Transform( tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(t.x(), t.y(), t.z()) );
}

} // end namespace utils