#include <hector_pose_prediction_benchmark_tools/pose_prediction_benchmark.h>

#include <hector_pose_prediction_benchmark_tools/util.h>
#include <hector_pose_prediction_benchmark_tools/bag_reader.h>
#include <hector_stability_metrics/metrics/force_angle_stability_measure.h>
#include <ros/console.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>
#include <fstream>

namespace hector_pose_prediction_benchmark_tools {
PosePredictionBenchmark::PosePredictionBenchmark(
    hector_pose_prediction_interface::PosePredictor<double>::Ptr pose_predictor)
: pose_predictor_(std::move(pose_predictor)), wait_time_(0.0)
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  robot_model_ = robot_model_loader.getModel();
  if (!robot_model_.get()){
    ROS_ERROR("Could not load robot model!");
  }

  ros::NodeHandle nh("~");
  all_robot_states_pub_ = nh.advertise<visualization_msgs::MarkerArray>("all_robot_states", 10, true);
  current_robot_state_pub_ = nh.advertise<visualization_msgs::MarkerArray>("current_robot_state", 10, false);
}

void PosePredictionBenchmark::evaluate(const nav_msgs::Path& path, const std::vector<JointPositionMap>& joint_positions, bool init_from_previous)
{
  data_.clear();
  if (path.poses.size() != joint_positions.size()) {
    ROS_ERROR_STREAM("Path size (" << path.poses.size() << ") does not match joint position vector size (" << joint_positions.size() << ").");
    return;
  }

  bool first_iteration = true;
  hector_math::Pose<double> previous_predicted_pose;
  visualization_msgs::MarkerArray marker_array;
  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    const geometry_msgs::PoseStamped& pose_msg = path.poses[i];
    const JointPositionMap& joint_position = joint_positions[i];
    pose_predictor_->robotModel()->updateJointPositions(joint_position);
    DataPoint data_point;
    data_point.time = pose_msg.header.stamp;
    data_point.gt_pose = poseMsgToHectorMath(pose_msg.pose);

    // Estimate stability from input pose
    if (pose_predictor_->estimateContactInformation(data_point.gt_pose, data_point.estimated_support_polygon, data_point.estimated_contact_information)
        && !data_point.estimated_support_polygon.contact_hull_points.empty()) {
      Eigen::Vector3d com_base = pose_predictor_->robotModel()->centerOfMass();
      Eigen::Vector3d com_world = data_point.gt_pose * com_base;
      Eigen::Vector3d gravity(0, 0, -9.81);
      data_point.estimated_stability = hector_stability_metrics::non_differentiable::computeForceAngleStabilityMeasureValue(data_point.estimated_support_polygon.contact_hull_points, data_point.estimated_support_polygon.edge_stabilities, com_world, gravity);
    } else {
      data_point.estimated_stability = std::nan("");
    }

    // Erase pose information that we want to estimate
    hector_math::Pose<double> pose;
    Eigen::Vector3d rpy = rotationToEulerAngles(data_point.gt_pose.asTransform().linear());
    if (!first_iteration && init_from_previous) {
      Eigen::Vector3d previous_rpy = rotationToEulerAngles(previous_predicted_pose.asTransform().linear());
      pose =  hector_math::Pose<double>(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ())
                                       * Eigen::AngleAxisd(previous_rpy(1), Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(previous_rpy(0), Eigen::Vector3d::UnitX()));
      pose.translation() = data_point.gt_pose.translation();
      pose.translation().z() = previous_predicted_pose.translation().z();
    } else {
      pose = hector_math::Pose<double>(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
      pose.translation() = data_point.gt_pose.translation();
      pose.translation().z() = 0; // arbitrary initialization
    }

    // Pose prediction
    data_point.input_pose = pose;
    data_point.predicted_stability = pose_predictor_->predictPoseAndContactInformation(
        pose, data_point.predicted_support_polygon, data_point.predicted_contact_information);
    data_point.predicted_pose = pose;
    previous_predicted_pose = pose;
    first_iteration = false;

    // Marker array
    visualization_msgs::MarkerArray current_marker;
    addRobotMarkerToArray(data_point.gt_pose, joint_position, {0.0, 1.0, 1.0, 1.0}, "ground_truth", current_marker);
    addRobotMarkerToArray(data_point.input_pose, joint_position, {0.0, 1.0, 0.0, 1.0}, "input", current_marker);
    addRobotMarkerToArray(data_point.predicted_pose, joint_position, {0.0, 0.0, 1.0, 1.0}, "predicted", current_marker);
    current_robot_state_pub_.publish(current_marker);
    marker_array.markers.insert(marker_array.markers.end(), current_marker.markers.begin(), current_marker.markers.end());

    data_.push_back(std::move(data_point));
    ros::WallDuration(wait_time_).sleep();
  }
  fixMarkerIds(marker_array);
  all_robot_states_pub_.publish(marker_array);
}

bool PosePredictionBenchmark::saveToCsv(const std::string& csv_file_path) const
{
  if (!createParentDirectory(csv_file_path)) {
    return false;
  }
  if (data_.empty()) {
    return true;
  }

  ros::Time start_time = data_.front().time;

  std::ofstream file;
  file.open(csv_file_path);

  file << "time,"
       << getPoseLabels("ground_truth")
       << getPoseLabels("input")
       << "estimated_stability,"
       << getPoseLabels("predicted")
       << "predicted_stability"
       << std::endl;
  for (const auto& data_point: data_) {
    ros::Duration duration_since_start = data_point.time - start_time;
    file << duration_since_start.toSec() << ", ";
    file << poseToText(data_point.gt_pose) << ", ";
    file << poseToText(data_point.input_pose) << ", ";
    file << data_point.estimated_stability << ", ";
    file << poseToText(data_point.predicted_pose) << ", ";
    file << data_point.predicted_stability << std::endl;
  }

  file.close();
  return true;
}

void PosePredictionBenchmark::evaluateFromBag(const std::string& bag_path, double path_resolution, bool init_from_previous)
{
  BagReader reader(bag_path, pose_predictor_->robotModel()->jointNames());
  reader.setPathSamplingResolution(path_resolution);

  std::vector<JointPositionMap> joint_positions;
  nav_msgs::Path path;
  reader.parse(path,joint_positions);

  evaluate(path, joint_positions, init_from_previous);
}

void PosePredictionBenchmark::addRobotMarkerToArray(const hector_math::Pose<double>& pose,
                                                    const JointPositionMap& joint_positions,
                                                    const Eigen::Vector4f& rgba,
                                                    const std::string& ns,
                                                    visualization_msgs::MarkerArray& array)
{
  moveit::core::RobotState state(robot_model_);
  state.setVariablePositions(std::map<std::string, double>(joint_positions.begin(), joint_positions.end()));
  state.setVariablePosition("world_virtual_joint/trans_x", pose.translation().x());
  state.setVariablePosition("world_virtual_joint/trans_y", pose.translation().y());
  state.setVariablePosition("world_virtual_joint/trans_z", pose.translation().z());
  state.setVariablePosition("world_virtual_joint/rot_x", pose.orientation().x());
  state.setVariablePosition("world_virtual_joint/rot_y", pose.orientation().y());
  state.setVariablePosition("world_virtual_joint/rot_z", pose.orientation().z());
  state.setVariablePosition("world_virtual_joint/rot_w", pose.orientation().w());

  std_msgs::ColorRGBA color;
  color.r = rgba(0);
  color.g = rgba(1);
  color.b = rgba(2);
  color.a = rgba(3);
  state.getRobotMarkers(array, robot_model_->getLinkModelNames(), color, ns, ros::Duration(0));
}

double PosePredictionBenchmark::getWaitTime() const
{
  return wait_time_;
}

void PosePredictionBenchmark::setWaitTime(double seconds)
{
  wait_time_ = seconds;
}

}
