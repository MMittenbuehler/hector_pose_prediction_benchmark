#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H

#include <hector_pose_prediction_interface/types.h>
#include <geometry_msgs/Pose.h>
#include <filesystem>

namespace hector_pose_prediction_benchmark_tools {

hector_math::Pose<double> poseMsgToHectorMath(const geometry_msgs::Pose& pose_msg);

bool createParentDirectory(const std::string& file_path);

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_UTIL_H
