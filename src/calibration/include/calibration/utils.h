#ifndef _UTILS_H_
#define _UTILS_H_

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <cstdint>
#include <ros/serialization.h>

namespace utils {

// NOTE: These are taken from the multi_pose_graph repo.
// Duplicate code could probably be better avoided, but simpler this way for now

Eigen::Affine3d rosPoseToEigen(const geometry_msgs::Pose& ros_pose);

geometry_msgs::Pose eigenToRosPose(const Eigen::Affine3d& T_aff);

Eigen::Affine3d tfToEigen(const tf::Transform& tf);

tf::Transform eigenToTf(const Eigen::Affine3d& T_aff);

template <typename T>
T bytesToMsg(std::vector<uint8_t>& buffer) {
  ros::serialization::IStream stream(buffer.data(), buffer.size());
  T ros_msg;
  ros::serialization::deserialize(stream, ros_msg);
  return ros_msg;
}

template <typename T>
std::vector<uint8_t> msgToBytes(const T& msg) {
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  
  std::vector<uint8_t> buffer(serial_size);
  ros::serialization::OStream stream(buffer.data(), serial_size);
  ros::serialization::serialize(stream, msg);

  return buffer;
}


} // end namespace utils

#endif