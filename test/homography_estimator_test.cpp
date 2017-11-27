#include <gtest/gtest.h>

#define private public
#include "drive_ros_camera_homography/homography_estimator.h"

// Declare test
TEST(HomographyEstimator, testCase0)
{

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  HomographyEstimator he(nh, pnh);

  cv::Mat mat;
  mat.push_back(cv::Mat(1,5,CV_64F,3.5));
  mat.push_back(cv::Mat(1,5,CV_64F,9.1));
  mat.push_back(cv::Mat(1,5,CV_64F,2.7));

// matrix will look like:
//  [3.5, 3.5, 3.5, 3.5, 3.5;
//   9.1, 9.1, 9.1, 9.1, 9.1;
//   2.7, 2.7, 2.7, 2.7, 2.7]

  // inside borders
  cv::Point p1(0,0);
  EXPECT_EQ(p1, he.checkPointSize(mat, p1.x, p1.y));

  cv::Point p2(4,3);
  EXPECT_EQ(p2, he.checkPointSize(mat, p2.x, p2.y));

  // outside of borders
  cv::Point p3(-1,-1);
  EXPECT_EQ(cv::Point(0,0), he.checkPointSize(mat, p3.x, p3.y));

  cv::Point p4(1000,1000);
  EXPECT_EQ(cv::Point(mat.cols,mat.rows), he.checkPointSize(mat, p4.x, p4.y));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "homography_tester");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
