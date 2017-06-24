#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

static const std::string OPENCV_WINDOW = "Image window";

class CameraHomography
{
  ros::NodeHandle nh_;
  ros::Subscriber cam_info_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  bool calLoaded;
  bool homCalculated;
  image_geometry::PinholeCameraModel camera_model_; 
  cv::Mat homography;
  std::vector<double> d; 
  std::vector<cv::Point3f> patternPoints_;
  double length;
  double xoff;
  double yoff;
  cv::Size patternSize_;
public:
  CameraHomography()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera1/image_raw", 1,
      &CameraHomography::imageCb, this);
    cam_info_sub = nh_.subscribe("/camera1/camera_info", 1, &CameraHomography::camInfoCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    homCalculated = false;
    patternSize_ = cv::Size(4, 15);
    length = 6;
    xoff = 21.1;
    yoff = 45.3;
    calLoaded = false;
    patternPoints_ = std::vector<cv::Point3f>(); 

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~CameraHomography()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void camInfoCb(const sensor_msgs::CameraInfo& info){
      if(!calLoaded){
        camera_model_.fromCameraInfo(info);	
        calLoaded = true;
	std::cout << camera_model_.distortionCoeffs() << std::endl;
      }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if(homCalculated) return;

    cv_bridge::CvImagePtr cv_ptr;
    std::vector<cv::Point2f> points;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat res;
    if (calLoaded){
      cv::undistort(cv_ptr->image, res, camera_model_.intrinsicMatrix(), camera_model_.distortionCoeffs());
      bool found = false;
      found = cv::findCirclesGrid(res, patternSize_, points, cv::CALIB_CB_ASYMMETRIC_GRID);
      std::cout << found << std::endl;
      cv::drawChessboardCorners(res, patternSize_, cv::Mat(points), found);

      

      
      for(int i = 0; i < patternSize_.height; i++) {
        for (int j = 0; j < patternSize_.width; j++) {
          patternPoints_.emplace_back(xoff + ((2 * j + i % 2) * length), yoff + (i * length), 0);
        }
      }
      res.copyTo(cv_ptr->image);
    }
    

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
 
  void calculatePatternPoint(){

  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_homography");
  CameraHomography ch;
  ros::spin();
  return 0;
}

