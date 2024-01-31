#include <hector_pose_prediction_benchmark/pose_predictor_factory.h>

#include <sdf_contact_estimation/sdf_contact_estimation.h>
#include <hector_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <hector_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
#include <hector_world_heightmap/io.h>
#include <gpu_heightmap_pose_prediction/heightmap_pose_predictor.h>
#include <gpu_heightmap_pose_prediction/robot_heightmap_providers/urdf_robot_heightmap_provider.h>
//#include <hector_world_heightmap_ros/message_conversions/map.h>

namespace hector_pose_prediction_benchmark {

hector_pose_prediction_interface::PosePredictor<double>::Ptr createPosePredictor(const std::string& name, const ros::NodeHandle& nh) {
  if (name == "sdf_contact_estimation") {
    ros::NodeHandle pnh(nh, "sdf_contact_estimation");
    return createSdfContactEstimation(pnh);
  } else if (name == "hector_heightmap_pose_prediction") {
      ros::NodeHandle pnh(nh, "hector_heightmap_pose_prediction");
      return createHeightmapPosePrediction(pnh);
  } else if (name == "gpu_heightmap_pose_prediction") {
      ros::NodeHandle pnh(nh, "gpu_heightmap_pose_prediction");
      return createGpuPosePrediction(pnh);
  }
  ROS_ERROR_STREAM("Unknown pose predictor " << name);
  return {};
}

hector_pose_prediction_interface::PosePredictor<double>::Ptr createSdfContactEstimation(const ros::NodeHandle& nh) {
  // Map
  ros::NodeHandle sdf_model_nh(nh, "sdf_map");
  auto sdf_model = std::make_shared<sdf_contact_estimation::SdfModel>(sdf_model_nh);
  sdf_model->loadFromServer(sdf_model_nh);
  // Robot model
  ros::NodeHandle shape_model_nh(nh, "shape_model");
  auto shape_model = std::make_shared<sdf_contact_estimation::ShapeModel>(shape_model_nh);
  // Predictor
  auto sdf_pose_predictor = std::make_shared<sdf_contact_estimation::SDFContactEstimation>(nh, shape_model, sdf_model);
  sdf_pose_predictor->enableVisualisation(false);
  return std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(sdf_pose_predictor);
}

hector_pose_prediction_interface::PosePredictor<double>::Ptr createHeightmapPosePrediction(const ros::NodeHandle& nh) {
  // Heightmap
  ros::NodeHandle heightmap_nh(nh, "heightmap");
  std::string map_path;
  if (!heightmap_nh.getParam("map_bag_file_path", map_path)) {
    ROS_ERROR_STREAM("Could not load mandatory parameter '" << heightmap_nh.getNamespace() << "/map_bag_file_path'");
    return {};
  }
  hector_world_heightmap::MapBag<double>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<double>(map_path);
  if (!map_bag) {
    ROS_ERROR_STREAM("Failed to load map from path '" << map_path << "'");
    return {};
  }
//  heightmap_pub.publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
  // Robot Model
  urdf::Model urdf;
  if (!urdf.initParam("/robot_description")) {
    ROS_ERROR_STREAM("Failed to load urdf.");
    return {};
  }
  auto robot_heightmap_provider = std::make_shared<hector_heightmap_pose_prediction::UrdfRobotHeightmapProvider<double>>(map_bag->resolution(), urdf);
  robot_heightmap_provider->disableHeightmapCache();
  // Pose predictor
  auto heightmap_pose_predictor = std::make_shared<hector_heightmap_pose_prediction::HeightmapPosePredictor<double>>(map_bag, robot_heightmap_provider);
  return std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
}


hector_pose_prediction_interface::PosePredictor<double>::Ptr createGpuPosePrediction(const ros::NodeHandle& nh) {
    // Heightmap
    ros::NodeHandle heightmap_nh(nh, "heightmap");
    std::string map_path;
    if (!heightmap_nh.getParam("map_bag_file_path", map_path)) {
        ROS_ERROR_STREAM("Could not load mandatory parameter '" << heightmap_nh.getNamespace() << "/map_bag_file_path'");
        return {};
    }
    hector_world_heightmap::MapBag<float>::Ptr map_bag = hector_world_heightmap::io::loadFromFile<float>(map_path);
    if (!map_bag) {
        ROS_ERROR_STREAM("Failed to load map from path '" << map_path << "'");
        return {};
    }
//  heightmap_pub.publish(hector_world_heightmap_ros::message_conversions::heightmapToMsg<double>(map_bag->getMap(0)));
    // Robot Model
    urdf::Model urdf;
    if (!urdf.initParam("/robot_description")) {
        ROS_ERROR_STREAM("Failed to load urdf.");
        return {};
    }
    auto robot_heightmap_provider = std::make_shared<gpu_heightmap_pose_prediction::UrdfRobotHeightmapProvider>(map_bag->resolution(), urdf);
//    robot_heightmap_provider->disableHeightmapCache();
    // Pose predictor
//    gpu_heightmap_pose_prediction::HeightmapPosePredictor<false>* posePredictor = new gpu_heightmap_pose_prediction::HeightmapPosePredictor<false>(map_bag, robot_heightmap_provider);
    auto heightmap_pose_predictor = std::make_shared<gpu_heightmap_pose_prediction::HeightmapPosePredictor<false>>(map_bag, robot_heightmap_provider);
    return std::static_pointer_cast<hector_pose_prediction_interface::PosePredictor<double>>(heightmap_pose_predictor);
}


}
