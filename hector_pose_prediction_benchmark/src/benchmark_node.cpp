#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_prediction_benchmark_suite");

  ros::spin();
  return 0;
}

