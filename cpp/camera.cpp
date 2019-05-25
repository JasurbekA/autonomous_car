#include "opencv2/opencv.hpp"
#include "../headers/camera_lib.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

CameraManager::CameraManager() {
  cascade_pedestrian.load("/home/pi/Project/Final/cpp/cascade/pedestrian.xml");
	cascade_left.load("/home/pi/Project/Final/cpp/cascade/left.xml");
	cascade_stop.load("/home/pi/Project/Final/cpp/cascade/stop.xml");
	cascade_parking.load("/home/pi/Project/Final/cpp/cascade/parking.xml");
	cascade_right.load("/home/pi/Project/Final/cpp/cascade/right.xml");
 
}

void CameraManager::detectSign(cv::Mat& upper_roi) {
   cascade_stop.detectMultiScale(upper_roi, stop_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 60));
   cascade_left.detectMultiScale(upper_roi, left_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(40, 40));
   cascade_right.detectMultiScale(upper_roi, right_, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));
   cascade_pedestrian.detectMultiScale(upper_roi, pedestrian, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(60, 60));
   cascade_parking.detectMultiScale(upper_roi, parking, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(80, 80));
   
   
    if(stop_.size() > 0) {
      detectedSign = TrafficSign::STOP_SIGN;
    }else if(left_.size() > 0) {
      detectedSign = TrafficSign::LEFT_SIGN;    
    }else if (right_.size() > 0) {
      detectedSign = TrafficSign::RIGHT_SIGN;    
    }else if(pedestrian.size() > 0) {
      detectedSign = TrafficSign::PEDESTRIAN;      
    }else if(parking.size() > 0){
      detectedSign = TrafficSign::PARKING;
    }else if(isTrafficLightRedDetected(upper_roi)) {
      detectedSign = TrafficSign::TRAFFIC_LIGHT_RED;
    }else if (isTrafficLightGreenDetected(upper_roi)) {
      detectedSign = TrafficSign::TRAFFIC_LIGHT_GREEN;
    }else{
      detectedSign = TrafficSign::NO_SIGN;
    }
    
}

void CameraManager::detectLines(cv::Mat& frame) {
    src_roi_left = frame(roi_left);
    src_roi_right = frame(roi_right);
    
    //Extarct only yellow color    
    inRange(src_roi_left, cv::Scalar(0, 169, 185), cv::Scalar(255, 255, 255), src_roi_left);
    inRange(src_roi_right, cv::Scalar(0, 169, 185), cv::Scalar(255, 255, 255), src_roi_right);
    

    Canny(src_roi_left, dst_l, 50, 200);
    Canny(src_roi_right, dst_r, 50, 200);

    cvtColor(dst_l, cdst_l, cv::COLOR_BGR2GRAY);
    cvtColor(dst_r, cdst_r, cv::COLOR_BGR2GRAY);

    
    HoughLinesP(dst_r, right_side_lines, 1, CV_PI / 60, 150, 20, 10); //finds lines in image(binary form)
    HoughLinesP(dst_l, left_side_lines, 1, CV_PI / 60, 150, 20, 10); //finds lines in image(binary form)

    hasLeftLine = left_side_lines.size() > 0;
    hasRightLine = right_side_lines.size() > 0;
    

    if (hasLeftLine && hasRightLine){
     // go forward

       if (hasLeftDisappeared) {
            hasLeftDisappeared = false;
            // turn a right litte and go forward
             direction = Direction::TURN_RIGHT_LITTLE;
       }
       if (hasRightDisappeared) {
            hasRightDisappeared = false;
            // turn left a litte and go forward
            direction = Direction::TURN_LEFT_LITTLE;
       }

       angle_left = getAngle(left_side_lines[left_side_lines.size() - 1]);
       angle_right = getAngle(right_side_lines[0]);
       
      
       if (angle_left < 0 && angle_right > 0) {
          //Road is straight (maybe small midification needed)
          if (isRightModificationNeeded()) {
            // Right a little and go
             direction = Direction::TURN_RIGHT_LITTLE;

          }else if (isLeftModificationNeeded()){
            // Left a little and go
            direction = Direction::TURN_LEFT_LITTLE;
          }
            
       }else if (angle_left > 0 && angle_right > 0) {
          //turn right 
            direction = Direction::TURN_RIGHT_SHARPER;
            
       }else if (angle_left < 0 && angle_right < 0) {
          // turn left
            direction = Direction::TURN_LEFT_SHARPER;
       }

    }else  if (hasRightLine && !hasLeftLine){
     // no lines at the left side, therefore we need to move left
     hasLeftDisappeared = true;
     direction = Direction::TURN_LEFT_SHARPER; 
    }else if (hasLeftLine && !hasRightLine){
     // no lines at the right side, therefore we need to move right
     hasRightDisappeared = true;
     direction = Direction::TURN_RIGHT_SHARPER;
    }else {
     // both sides no line then go forward
     direction = Direction::GO_FORWARD;
    }

}

bool CameraManager::isTrafficLightRedDetected(cv::Mat& upper_roi) {
      cv::Mat hsv;
      cvtColor(upper_roi, hsv, cv::COLOR_BGR2HSV);
     
      cv::Mat lower_red_threashold, upper_red_threashold;
      inRange(hsv, cv::Scalar(0, 0, 116), cv::Scalar(7, 255, 255), lower_red_threashold);
      inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), upper_red_threashold);


      // Combine the above two color
      cv::Mat red_image;
      addWeighted(lower_red_threashold, 1.0, upper_red_threashold, 1.0, 0.0, red_image);
      erode(red_image, red_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9)));
      dilate(red_image, red_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
      


      std::vector<cv::Vec3f> number_of_red_circles;
      HoughCircles(red_image, number_of_red_circles, cv::CV_HOUGH_GRADIENT, 1, red_image.rows/8, 100, 20, 1/*min radius*/, 500/*max radius*/);

      return number_of_red_circles.size() > 0; 
}

bool CameraManager::isTrafficLightGreenDetected(cv::Mat& upper_roi){
      cv::Mat hsv;
      cvtColor(upper_roi, hsv, cv::COLOR_BGR2HSV);
     
      cv::Mat lower_green_threashold, upper_green_threashold;
      inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(45, 100, 50), lower_green_threashold);
      inRange(hsv, cv::Scalar(50, 100, 100), cv::Scalar(75, 255, 255), upper_green_threashold);


      // Combine the above two color
      cv::Mat green_image;
      addWeighted(lower_green_threashold, 1.0, upper_green_threashold, 1.0, 0.0, green_image);
      erode(green_image, green_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9)));
      dilate(green_image, green_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
      


      std::vector<cv::Vec3f> number_of_green_circles;
      HoughCircles(green_image, number_of_green_circles, cv::CV_HOUGH_GRADIENT, 1, green_image.rows/8, 100, 20, 1/*min radius*/, 500/*max radius*/);

      return number_of_green_circles.size() > 0; 
}

bool CameraManager::isRightModificationNeeded() {
  return fabs(angle_left) - angle_right >= angle_threshold;
}

bool CameraManager::isLeftModificationNeeded() {
  return fabs(angle_left) + angle_right <= angle_threshold;
}

float CameraManager::getAngle (Vec4i line) {
    
 lineStart = Point(line[0], line[1]);
 lineEnd = Point(line[2], line[3]);

 angle = atan2(lineEnd.y - lineStart.y, lineEnd.x - lineStart.x);
 angle = angle * 180 / 3.14;

 cout << angle << "\n";
 
 return angle;
}

