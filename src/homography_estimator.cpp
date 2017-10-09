#include "drive_ros_camera_homography/homography_estimator.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


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

// constructor
HomographyEstimator::HomographyEstimator(const ros::NodeHandle nh, const ros::NodeHandle pnh):
  nh_(nh),
  it_(pnh),
  pnh_(pnh)
{
  // subscribe to topics
  image_sub_ = it_.subscribe("img_in", 1, &HomographyEstimator::imageCb, this);
  cam_info_sub = nh_.subscribe("cam_info", 1, &HomographyEstimator::camInfoCb, this);

  // launch CV windows
  cv::namedWindow("homography_estimator_input");
  cv::namedWindow("homography_estimator_keypoints", cv::WINDOW_NORMAL);
  cv::namedWindow("homography_estimator_pattern", cv::WINDOW_NORMAL);
  cv::namedWindow("homography_estimator_preview");

  ROS_INFO("connecting to dynamic reconfiguration server");
  ros::NodeHandle reconf_node(pnh_, "settings");

  // bind dynamic reconfigure callback
  dyn_rec_cbtype = boost::bind(&HomographyEstimator::reconfigureCB, this, _1, _2);
  dyn_rec_server.setCallback(dyn_rec_cbtype);

}

// desctructor
HomographyEstimator::~HomographyEstimator() {
    //destroy CV windows
    cv::destroyWindow("homography_estimator_input");
    cv::destroyWindow("homography_estimator_pattern");
    cv::destroyWindow("homography_estimator_keypoints");
    cv::destroyWindow("homography_estimator_preview");
}


void HomographyEstimator::reconfigureCB(drive_ros_camera_homography::homography_estimatorConfig& config, uint32_t level)
{
  cfg = config;
  ROS_INFO_STREAM("update configs from reconfigure");
}

// initialize parameters
void HomographyEstimator::initParameters() {

    // setup pattern
    pattern = static_cast<Pattern>(cfg.pattern);
    patternSize = cv::Size(cfg.points_per_row, cfg.points_per_col);
    computePatternPoints();

    // get another parameter
    outlineScaleFactor = cfg.outline_scale_factor;

    // initialize blob detector
    auto blobParams = getBlobDetectorParams();
    blobDetector = cv::SimpleBlobDetector::create(blobParams);

    return;
}



// compute pattern points based on pattern, size, offset, ...
void HomographyEstimator::computePatternPoints()
{
    worldPoints.resize(0);

    float length = cfg.pattern_length;
    float offset_x = cfg.pattern_offset_x;
    float offset_y = cfg.pattern_offset_y;

    switch (pattern) {
        case Pattern::CHESSBOARD:
        case Pattern::CIRCLES:
            for (int i = 0; i < patternSize.height; i++) {
                for (int j = 0; j < patternSize.width; j++) {
                    worldPoints.emplace_back(offset_x + i * length,
                                             offset_y + j * length);
                }
            }
            break;
        case Pattern::CIRCLES_ASYMMETRIC:
            for (int i = 0; i < patternSize.height; i++) {
                for (int j = 0; j < patternSize.width; j++) {
                    worldPoints.emplace_back(offset_x + i * length,
                                             offset_y + (2 * j + i % 2) * length);
                }
            }
            break;
    }
}

void HomographyEstimator::camInfoCb(const sensor_msgs::CameraInfo& info){
    if(!calLoaded){
      camera_model_.fromCameraInfo(info);
      calLoaded = true;
      std::cout << camera_model_.distortionCoeffs() << std::endl;
      ROS_INFO_STREAM("Got camera info message. Saving Parameters.");
    }
}



void HomographyEstimator::imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  // prevent from reloading every time a new image arrives and check if we have a calibration loaded
  if(!calLoaded || imgLoaded){
    return;
  }
  imgLoaded = true;
  ROS_INFO_STREAM("New image available. Trying again.");

  // give dynamic reconfigure some time to apply new parameters
  ros::Duration(1.0).sleep();

  // reload paramters on every image so that the user can change them
  initParameters();

  // process image
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img, imgColor;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    imgLoaded = false;
    return;
  }
  img = cv_ptr->image;
  cv::cvtColor(img, imgColor, CV_GRAY2BGR);

  // get estimates points from cfg
  std::vector<cv::Point2f> estimatePoints;
  estimatePoints.emplace_back(cfg.p_top_left_x, cfg.p_top_left_y);
  estimatePoints.emplace_back(cfg.p_top_right_x, cfg.p_top_right_y);
  estimatePoints.emplace_back(cfg.p_bot_right_x, cfg.p_bot_right_y);
  estimatePoints.emplace_back(cfg.p_bot_left_x, cfg.p_bot_left_y);

  line(imgColor, cv::Point(cfg.p_bot_right_x, cfg.p_bot_right_y),
                 cv::Point(cfg.p_bot_left_x, cfg.p_bot_left_y),
                 cv::Scalar(110, 220, 0),  2, 8 );

  line(imgColor, cv::Point(cfg.p_bot_left_x, cfg.p_bot_left_y),
                 cv::Point(cfg.p_top_left_x, cfg.p_top_left_y),
                 cv::Scalar(110, 220, 0),  2, 8 );

  line(imgColor, cv::Point(cfg.p_top_left_x, cfg.p_top_left_y),
                 cv::Point(cfg.p_top_right_x, cfg.p_top_right_y),
                 cv::Scalar(110, 220, 0),  2, 8 );

  line(imgColor, cv::Point(cfg.p_top_right_x, cfg.p_top_right_y),
                 cv::Point(cfg.p_bot_right_x, cfg.p_bot_right_y),
                 cv::Scalar(110, 220, 0),  2, 8 );

  cv::imshow("homography_estimator_input", imgColor);

  // pattern outline
  std::vector<cv::Point2f> outlinePoints;
  cv::Size outlineSize = (patternSize + cv::Size(2, 2)) * outlineScaleFactor;
  outlinePoints.clear();
  outlinePoints.emplace_back(0, 0);
  outlinePoints.emplace_back(outlineSize.width, 0);
  outlinePoints.emplace_back(outlineSize.width, outlineSize.height);
  outlinePoints.emplace_back(0, outlineSize.height);

  // calculate estimate
  estimate = cv::findHomography(estimatePoints, outlinePoints);

  if(!estimate.empty()){
      // Warp image using estimate
      cv::Mat warped, tmp, warpedColor;
      // Warp the logo image to change its perspective
      cv::warpPerspective(img, tmp, estimate, outlineSize);
      cv::equalizeHist(tmp, warped);
      cv::cvtColor(warped, warpedColor, CV_GRAY2BGR);

      // Detect keypoints
      {
          std::vector<cv::KeyPoint> keypoints;
          blobDetector->detect(warped, keypoints);
          cv::Mat detection;
          cv::drawKeypoints(warped, keypoints, detection);
          cv::imshow("homography_estimator_keypoints", detection);
      }

      // Detect pattern
      std::vector<cv::Point2f> detectedPoints;
      bool patternDetected = findPoints(warped, detectedPoints);
      cv::drawChessboardCorners(warpedColor, patternSize, cv::Mat(detectedPoints), patternDetected);


      // Detect Keypoints
      cv::imshow("homography_estimator_pattern", warpedColor);

      // Compute Refinement
      if(patternDetected){
          if(computeRefinement(detectedPoints)){
              // Compute Final homography (cam2world)
              cam2world = refinement * estimate;

              world2cam = cam2world.inv();
              ROS_INFO_STREAM("cam2world" << cam2world);
              ROS_INFO_STREAM("world2cam" << world2cam);

              // Save to file
              saveHomography();

              // Compute top view
              computeTopView();
              // Show preview
              cv::Mat topView;
              cv::warpPerspective(img, topView, topView2cam.inv(), topViewSize);
              cv::imshow("homography_estimator_preview", topView);

              ROS_INFO_STREAM("Homography computed. Abort Program or start over with another image (press any key).");
              cv::waitKey(0);

          }
      }else{
          ROS_ERROR("Pattern not detected! You may want to change parameters and give it another try!");
      }
  }

  cv::waitKey(10);
  imgLoaded = false;
  return;
}


cv::SimpleBlobDetector::Params HomographyEstimator::getBlobDetectorParams()
{
    // Blob detector
    cv::SimpleBlobDetector::Params blobParams;
    blobParams.filterByCircularity  = cfg.blob_filter_by_circularity;
    blobParams.minCircularity       = cfg.blob_min_circularity;
    blobParams.maxCircularity       = cfg.blob_max_circularity;

    blobParams.filterByConvexity    = cfg.blob_filter_by_convexity;
    blobParams.minConvexity         = cfg.blob_min_convexity;
    blobParams.maxConvexity         = cfg.blob_max_convexity;

    blobParams.filterByInertia      = cfg.blob_filter_by_inertia;
    blobParams.minInertiaRatio      = cfg.blob_min_inertia;
    blobParams.maxInertiaRatio      = cfg.blob_max_inertia;

    blobParams.filterByArea         = cfg.blob_filter_by_area;
    blobParams.minArea              = cfg.blob_min_area;
    blobParams.maxArea              = cfg.blob_max_area;
    return blobParams;
}


void HomographyEstimator::computeTopView()
{
    // Choose top-view image size
    topViewSize = cv::Size(512, 512);

    // Choose corner points (in real-world coordinates)
    std::vector<cv::Point2f> coordinates;
    coordinates.emplace_back(0, -1.500);
    coordinates.emplace_back(0, 1.500);
    coordinates.emplace_back(3.000, -1.500);
    coordinates.emplace_back(3.000, 1.500);

    std::vector<cv::Point2f> pixels;
    pixels.emplace_back(0, topViewSize.height);
    pixels.emplace_back(0, 0);
    pixels.emplace_back(topViewSize.width, topViewSize.height);
    pixels.emplace_back(topViewSize.width, 0);

    cv::Mat H = cv::findHomography(pixels, coordinates);

    topView2cam = world2cam * H;
    ROS_INFO_STREAM("topView2cam" << topView2cam);
}


bool HomographyEstimator::computeRefinement(const std::vector<cv::Point2f> detectedPoints){
    if(detectedPoints.size() != worldPoints.size()){
        ROS_ERROR_STREAM("computeRefinement"<<"point count doesn't match!"<<detectedPoints.size()<<" "<<worldPoints.size());
        return false;
    }
    refinement = cv::findHomography(detectedPoints, worldPoints);
    return true;
}




bool HomographyEstimator::findPoints(const cv::Mat& img, std::vector<cv::Point2f>& points)
{
    bool success = false;
    switch (pattern) {
        case Pattern::CHESSBOARD:
            success = cv::findChessboardCorners(img, patternSize, points);
            break;
        case Pattern::CIRCLES:
            success = cv::findCirclesGrid(img, patternSize, points, cv::CALIB_CB_SYMMETRIC_GRID, blobDetector);
            break;
        case Pattern::CIRCLES_ASYMMETRIC:
            success = cv::findCirclesGrid(img, patternSize, points, cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector);
            break;
    }
    return success;
}





void HomographyEstimator::saveHomography()
{

// TODO: implement some fancy saving function

//    // Save homography
//    if(!isEnableSave()) {
//        logger.error() << "Command line option --enable-save was not specified";
//        return;
//    }

//    if(cam2world.rows != 3 || cam2world.cols != 3) {
//        logger.error("cam2world") << "Invalid homography matrix dimensions: " << cam2world.rows << " x " << cam2world.cols;
//        return;
//    }

//    std::ofstream out(saveLogDir("homography_estimator") + "homography.lconf");

//    out << "cam2world = ";
//    for(int r = 0; r < 3; r++) {
//        for(int c = 0; c < 3; c++) {
//            out << cam2world.at<double>(r, c);
//            if( r != 2 || c != 2) { //not ending with ,
//                out << ",";
//            }
//        }
//    }
//    out << std::endl;

//    out << "world2cam = ";
//    for(int r = 0; r < 3; r++) {
//        for(int c = 0; c < 3; c++) {
//            out << world2cam.at<double>(r, c);
//            if( r != 2 || c != 2) { //not ending with ,
//                out << ",";
//            }
//        }
//    }
//    out << std::endl;
}
