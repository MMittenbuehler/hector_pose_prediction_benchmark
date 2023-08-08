#include <hector_pose_prediction_benchmark_tools/util.h>

#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>

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

void updateJointPositionMap(const sensor_msgs::JointStateConstPtr& joint_state_msg,
                         std::unordered_map<std::string, double>& joint_state_map,
                         std::set<std::string>& missing_joint_states)
{
  if (joint_state_msg) {
    // Mark seen joints
    if (!missing_joint_states.empty()) {
      for (const auto & joint_name : joint_state_msg->name) {
        auto it = missing_joint_states.find(joint_name);
        if (it != missing_joint_states.end()) {
          missing_joint_states.erase(it);
        }
      }
    }
    // Update state
    for (unsigned int i = 0; i < joint_state_msg->name.size(); ++i) {
      joint_state_map[joint_state_msg->name[i]] = joint_state_msg->position[i];
    }
  }
}

bool addPoseToPath(const geometry_msgs::TransformStamped& transform_msg, nav_msgs::Path& path, double sampling_distance) {
  Eigen::Isometry3d pose;
  tf::transformMsgToEigen(transform_msg.transform, pose);

  bool add_pose = false;
  if (path.poses.empty()) {
    add_pose = true;
  } else {
    Eigen::Vector3d previous_point;
    tf::pointMsgToEigen(path.poses.back().pose.position, previous_point);
    double distance = (previous_point - pose.translation()).norm();
    if (distance > sampling_distance) {
      add_pose = true;
    }
  }

  if (add_pose) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = transform_msg.header;
    tf::poseEigenToMsg(pose, pose_msg.pose);
    ROS_INFO_STREAM("Adding pose [" <<
                    pose_msg.pose.position.x << ", " <<
                    pose_msg.pose.position.y << ", " <<
                    pose_msg.pose.position.z << "].");
    path.poses.push_back(std::move(pose_msg));
  }
  return add_pose;
}

void fixMarkerIds(visualization_msgs::MarkerArray& array) {
  for (int i = 0; i < array.markers.size(); ++i) {
    array.markers[i].id = i;
  }
}

std::string poseToText(const hector_math::Pose<double>& pose) {
  std::stringstream ss;
  ss << pose.translation().x() << ", ";
  ss << pose.translation().y() << ", ";
  ss << pose.translation().z() << ", ";
  Eigen::Vector3d input_rpy = rotationToEulerAngles(pose.asTransform().linear());
  ss << input_rpy(0) << ", ";
  ss << input_rpy(1) << ", ";
  ss << input_rpy(2) ;
  return ss.str();
}

std::string getPoseLabels(const std::string& pose_name) {
  std::stringstream ss;
  ss << pose_name << "_position_x,";
  ss << pose_name << "_position_y,";
  ss << pose_name << "_position_z,";
  ss << pose_name << "_orientation_roll,";
  ss << pose_name << "_orientation_pitch,";
  ss << pose_name << "_orientation_yaw,";
  return ss.str();
}

}

