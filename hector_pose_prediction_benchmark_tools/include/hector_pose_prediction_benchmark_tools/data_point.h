#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_DATA_POINT_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_DATA_POINT_H

#include <Eigen/Eigen>
#include <hector_pose_prediction_interface/types.h>
#include <ros/time.h>

namespace hector_pose_prediction_benchmark_tools {

struct DataPoint
{
  ros::Time time;

  hector_math::Pose<double> gt_pose;
  hector_pose_prediction_interface::SupportPolygon<double> estimated_support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> estimated_contact_information;
  double estimated_stability; // Stability estimated from input_pose

  hector_math::Pose<double> input_pose;
  hector_math::Pose<double> predicted_pose;
  double predicted_stability;
  hector_pose_prediction_interface::SupportPolygon<double> predicted_support_polygon;
  hector_pose_prediction_interface::ContactInformation<double> predicted_contact_information;

  long prediction_time; // μs
};

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_TOOLS_DATA_POINT_H
