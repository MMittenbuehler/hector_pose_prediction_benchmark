#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H

#include <hector_pose_prediction_interface/types.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <filesystem>
#include <ros/publisher.h>

#include <hector_pose_prediction_benchmark_tools/types.h>

namespace hector_pose_prediction_benchmark_tools {

template <typename T>
std::string setToString(const std::set<T>& set) {
  std::stringstream ss;
  ss << "[";
  for (auto entry: set) {
    ss << entry << ",";
  }
  ss << "]";
  return ss.str();
}

hector_math::Pose<double> poseMsgToHectorMath(const geometry_msgs::Pose& pose_msg);

bool createParentDirectory(const std::string& file_path);

Eigen::Vector3d rotationToEulerAngles(const Eigen::Matrix3d &rot);

void updateJointPositionMap(const sensor_msgs::JointStateConstPtr& joint_state_msg,
                         std::unordered_map<std::string, double>& joint_state_map,
                         std::set<std::string>& missing_joint_states);

bool addPoseToPath(const geometry_msgs::TransformStamped& transform_msg, nav_msgs::Path& path, double sampling_distance);

void fixMarkerIds(visualization_msgs::MarkerArray& array);

std::string poseToText(const hector_math::Pose<double>& pose);

std::string getPoseLabels(const std::string& pose_name);

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H
