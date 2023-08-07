#include <hector_pose_prediction_benchmark_tools/util.h>

#include <ros/console.h>

namespace hector_pose_prediction_benchmark_tools {

hector_math::Pose<double> poseMsgToHectorMath(const geometry_msgs::Pose& pose_msg) {
  return {pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,
           pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y,
           pose_msg.orientation.z};
}

bool createParentDirectory(const std::string& file_path) {
  std::filesystem::path path(file_path);
  std::filesystem::path parent_directory = path.parent_path();
  try {
    std::filesystem::create_directories(parent_directory);
  } catch (const std::filesystem::filesystem_error& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  catch (const std::bad_alloc& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

Eigen::Vector3d rotationToEulerAngles(const Eigen::Matrix3d &rot) {
  double epsilon = 1e-12;
  double roll, pitch, yaw;
  pitch = std::atan2(-rot.data()[2], sqrt(rot.data()[0]*rot.data()[0] + rot.data()[1]*rot.data()[1]));
  if (std::abs(pitch) > (M_PI_2-epsilon)) {
    yaw = std::atan2(-rot.data()[3], rot.data()[4]);
    roll = 0.0;
  } else {
    roll = std::atan2(rot.data()[5], rot.data()[8]);
    yaw = std::atan2(rot.data()[1], rot.data()[0]);
  }
  return {roll, pitch, yaw};
}

}

