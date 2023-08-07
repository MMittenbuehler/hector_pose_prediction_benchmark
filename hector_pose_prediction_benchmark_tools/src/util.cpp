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

}

