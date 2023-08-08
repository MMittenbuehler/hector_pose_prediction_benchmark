#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_BAG_READER_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_BAG_READER_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>

#include <hector_pose_prediction_benchmark_tools/types.h>

namespace hector_pose_prediction_benchmark_tools {

class BagReader
{
public:
  explicit BagReader(std::string bag_path, std::vector<std::string> joint_names);
  bool parse(nav_msgs::Path& path, std::vector<JointPositionMap>& joint_positions);

  double getPathSamplingResolution() const;
  void setPathSamplingResolution(double resolution);
private:
  void updateTfBuffer(const rosbag::MessageInstance& msg);

  tf2_ros::Buffer tf_buffer_;
  std::string bag_path_;
  std::vector<std::string> joint_names_;
  double path_sampling_resolution_;
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_BAG_READER_H
