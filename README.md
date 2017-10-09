# Camera Homography Estimator
Estimates homography between camera and ground plane and save transformation parameters to a file.

## Usage
0. put your camera in front of a camera calibration pattern or use recorded *.bag file
1. adapt topic names to your environment
2. launch .launch file
3. fire up `rqt` and open dynamic reconfigure plugin
4. adjust parameters until blob detector finds all pattern points
5. parameter file will be saved to the directory specified in .launch file

## Parameters
- pattern type (supported: `chessboard`, `circles`, `circles_asymmetric`)
- estimated points for homography `p_{bot|top}_{right|left}_{x|y}` (border of green lines in `homography_estimator_input` image)
- size of pattern (`points_per_col` and `points_per_row`)
- distance between pattern (`pattern_length`)
- offset of lower right point (`pattern_offset_x` and `pattern_offset_y`)
- some scale factor for outline, just use 50 (`outline_scale_factor`) 
- [blob filter settings](https://www.learnopencv.com/blob-detection-using-opencv-python-c/)

![List of Parameters](https://github.com/tum-phoenix/drive_ros_camera_homography/blob/master/doc/rqt_dynamic_reconfigure.png)

## windows
### homography_estimator_input
shows the input image and the estimated points for homography (green square)
![homography_estimator_input.png](https://github.com/tum-phoenix/drive_ros_camera_homography/blob/master/doc/homography_estimator_input.png)

### homography_estimator_keypoints
shows the warped image and the blob filter detection results
![homography_estimator_keypoints.png](https://github.com/tum-phoenix/drive_ros_camera_homography/blob/master/doc/homography_estimator_keypoints.png)

### homography_estimator_pattern
shows the pattern detection results
![homography_estimator_pattern.png](https://github.com/tum-phoenix/drive_ros_camera_homography/blob/master/doc/homography_estimator_pattern.png)

### homography_estimator_preview
shows the finally calculated homography applied on the orignal image
![homography_estimator_preview.png](https://github.com/tum-phoenix/drive_ros_camera_homography/blob/master/doc/homography_estimator_preview.png)

## example output file
```
%YAML:1.0
---
world2cam: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1.6692432475121174e+02, -1.0473530728192594e+02,
       4.1899035694525743e+01, -5.3058063431231153e+00,
       -8.5485048630232763e-01, 3.0488767398247290e+01,
       2.5714915733718796e-01, 1.9354996942966881e-03,
       2.1857104647575391e-02 ]
cam2world: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 9.2773372432628046e-05, -2.8302943486492732e-03,
       3.7701732480210497e+00, -9.5001192175387123e-03,
       8.5086756092260935e-03, 6.3424138245340718e+00,
       -2.5022146305447759e-04, 3.2544990711281058e-02,
       8.3393416152725042e-01 ]
```
       
