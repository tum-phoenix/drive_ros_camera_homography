#ifndef HOMOGRAPHY_ESTIMATOR_H
#define HOMOGRAPHY_ESTIMATOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include "drive_ros_camera_homography/homography_estimatorConfig.h"


enum class Pattern {
    CHESSBOARD,
    CIRCLES,
    CIRCLES_ASYMMETRIC
};

/**
 * @brief homography_estimator
 **/
class HomographyEstimator {

public:
    HomographyEstimator(const ros::NodeHandle nh, const ros::NodeHandle pnh);
    ~HomographyEstimator();

protected:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cam_info_sub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    Pattern pattern;
    cv::Size patternSize;
    std::vector<cv::Point2f> worldPoints;

    cv::Mat estimate;
    cv::Mat refinement;
    cv::Mat cam2world;
    cv::Mat world2cam;
    cv::Mat topView2cam;
    cv::Size topViewSize;

    image_geometry::PinholeCameraModel camera_model_;

    bool calLoaded = false;
    bool imgLoaded = false;

    int outlineScaleFactor;

    void initParameters();

    void camInfoCb(const sensor_msgs::CameraInfo& info);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void computePatternPoints();

    void saveHomography();

    bool findPoints(const cv::Mat& img, std::vector<cv::Point2f>& points);

    bool computeRefinement(const std::vector<cv::Point2f> detectedPoints);
    void computeTopView();

    dynamic_reconfigure::Server<drive_ros_camera_homography::homography_estimatorConfig> dyn_rec_server;
    dynamic_reconfigure::Server<drive_ros_camera_homography::homography_estimatorConfig>::CallbackType dyn_rec_cbtype;
    void reconfigureCB(drive_ros_camera_homography::homography_estimatorConfig& config, uint32_t level);
    drive_ros_camera_homography::homography_estimatorConfig cfg;

    // Blob Detector
    cv::Ptr<cv::SimpleBlobDetector> blobDetector;
    cv::SimpleBlobDetector::Params getBlobDetectorParams();

};

#endif // HOMOGRAPHY_ESTIMATOR_H
