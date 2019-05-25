#ifndef _CAMERA_HEADER
#define _CAMERA_HEADER

#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>


class CameraManager {
 
 private:
  
   cv::Mat src_roi_left;
   cv::Mat src_roi_right; 
   cv::Mat dst_l, dst_r, cdst_r, cdst_l;

   cv::Rect roi_left(0, 240, 320, 240); 
   cv::Rect roi_right(320, 240, 315, 240); 

   vector<Vec4i> left_side_lines, right_side_lines;


  cv::CascadeClassifier cascade_pedestrian,  cascade_left, cascade_stop,cascade_parking, cascade_right, cascade_forward;
  std::vector<cv::Rect> stop_, left_, right_, pedestrian, parking;
  
  int angle_threshold = 3;


  float angle;
  Point lineStart, lineEnd;
  float angle_left;
  float angle_right;
  //fucntions
  bool isRightModificationNeeded();
  bool isLeftModificationNeeded();
  bool isTrafficLightRedDetected(cv::Mat& upper_roi);
  bool isTrafficLightGreenDetected(cv::Mat& upper_roi);

 public:
  enum TrafficSign {
    NO_SIGN = 0,
    STOP_SIGN = 1,
    LEFT_SIGN = 2,
    RIGHT_SIGN = 3,
    PEDESTRIAN = 4,
    PARKING = 5, 
    TRAFFIC_LIGHT_RED = 6,
    TRAFFIC_LIGHT_GREEN = 7
  };

  enum Direction {
    TURN_RIGHT_LITTLE = 0,
    TURN_LEFT_LITTLE = 1,
    TURN_RIGHT_SHARPER = 2,
    TURN_LEFT_SHARPER = 3,
    GO_FORWARD = 4
  };
  
  CameraManager();
  TrafficSign detectedSign;
  Direction direction;
  
  void detectSign(cv::Mat& upper_roi);

  void detectLines(cv::Mat& frame);

};

#endif
