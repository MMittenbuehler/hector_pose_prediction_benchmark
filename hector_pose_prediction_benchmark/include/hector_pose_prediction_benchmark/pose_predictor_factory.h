#ifndef HECTOR_POSE_PREDICTION_BENCHMARK_POSE_PREDICTOR_FACTORY_H
#define HECTOR_POSE_PREDICTION_BENCHMARK_POSE_PREDICTOR_FACTORY_H

#include <hector_pose_prediction_interface/pose_predictor.h>
#include <ros/node_handle.h>

namespace hector_pose_prediction_benchmark {

hector_pose_prediction_interface::PosePredictor<double>::Ptr createPosePredictor(const std::string& name, const ros::NodeHandle& nh);

hector_pose_prediction_interface::PosePredictor<double>::Ptr createSdfContactEstimation(const ros::NodeHandle& nh);

hector_pose_prediction_interface::PosePredictor<double>::Ptr createHeightmapPosePrediction(const ros::NodeHandle& nh);

}

#endif  // HECTOR_POSE_PREDICTION_BENCHMARK_POSE_PREDICTOR_FACTORY_H
