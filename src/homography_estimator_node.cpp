#include "drive_ros_camera_homography/homography_estimator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "road_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

#ifndef NDEBUG
  // give GDB time to attach
  ros::Duration(2.0).sleep();
#endif

  HomographyEstimator homography(nh,pnh);
  ROS_INFO("homography estimator node succesfully initialized");

  while (ros::ok()) {
    ros::spin();
  }
  return 0;
}
