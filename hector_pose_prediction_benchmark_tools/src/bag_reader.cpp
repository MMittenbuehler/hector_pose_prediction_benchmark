#include <hector_pose_prediction_benchmark_tools/bag_reader.h>

#include <sensor_msgs/JointState.h>
#include <hector_pose_prediction_benchmark_tools/util.h>

namespace hector_pose_prediction_benchmark_tools {

BagReader::BagReader(std::string bag_path, std::vector<std::string> joint_names)
: bag_path_(std::move(bag_path)), joint_names_(std::move(joint_names)), path_sampling_resolution_(0.05) {}

bool BagReader::parse(nav_msgs::Path& path, std::vector<JointPositionMap>& joint_positions)
{
  tf_buffer_.clear();
  rosbag::Bag bag;
  try {
    bag.open(bag_path_, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  std::vector<std::string> topics{"/tf", "/tf_static", "/joint_states"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::unordered_map<std::string, double> joint_position_map;
  std::set<std::string> missing_joint_states(joint_names_.begin(),
                                             joint_names_.end());
  path.header.frame_id = "world";
  for(const rosbag::MessageInstance& m: view) {
    // Handle joint state msg
    sensor_msgs::JointState::ConstPtr joint_state_msg = m.instantiate<sensor_msgs::JointState>();
    updateJointPositionMap(joint_state_msg, joint_position_map, missing_joint_states);

    // Handle tf message
    updateTfBuffer(m);

    // Add poses to path
    if (missing_joint_states.empty()) {
      geometry_msgs::TransformStamped transform_msg;
      try {
        transform_msg = tf_buffer_.lookupTransform(path.header.frame_id, "base_link",ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        continue;
      }
      if (addPoseToPath(transform_msg, path, path_sampling_resolution_)) {
        joint_positions.push_back(joint_position_map);
      }
    } else {
      ROS_WARN_STREAM("Missing joint states: " << setToString(missing_joint_states));
    }
  }
  bag.close();
  return true;
}

void BagReader::updateTfBuffer(const rosbag::MessageInstance& msg)
{
  tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
  if (tf_msg) {
    bool is_static = msg.getTopic() == "/tf_static";
    for (const auto& transform_msg: tf_msg->transforms) {
      ROS_DEBUG_STREAM("Added transform " << transform_msg.header.frame_id << " -> " << transform_msg.child_frame_id << " at time "
                      << (is_static ? "static" : std::to_string(transform_msg.header.stamp.toSec())));
      tf_buffer_.setTransform(transform_msg, "BagReader", is_static);
    }
  }
}
double BagReader::getPathSamplingResolution() const
{
  return path_sampling_resolution_;
}
void BagReader::setPathSamplingResolution(double resolution)
{
  path_sampling_resolution_ = resolution;
}

}
