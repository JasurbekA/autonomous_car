#include "opencv2/imgproc.hpp"
#include <iostream>
#include <wiringPi.h>
#include "opencv2/opencv.hpp"
#include "./headers/motor_lib.h"
#include "./headers/line_tracer.h"
#include "./headers/camera_lib.h"
#include <thread>
#include <mutex>
#include <raspicam/raspicam_cv.h>


#define LIMIT_DISTANCE  20
#define TRIG_PIN 28
#define ECHO_PIN 29
#define LEFT_IR_PIN 27
#define RIGHT_IR_PIN 26 



int initialSpeed = 40;

std::mutex motorLock;

int objectCounter = 0;


LineTracerController lineTracerManager;
MotorController mController;


void init();
//Line Tracer Thread function
void lineTracer();
void irObject();
 
int main(int argc, char *argv[]) {
  
  
    init();
  
    cv::Mat frame;
    cv::Mat upper_roi;

    cv::Rect upper_roi_r (0,0,320,120);
    
    CameraManager cameraManager;
   
    raspicam::RaspiCam_Cv capture; 
    capture.open();

    
   //Start the thread
    std::thread irThread(lineTracer);
    std::thread ultraSonicThread(irObject);
  
    while (true) {
  
         capture.grab();
         capture.retrieve(frame);
         resize(frame, frame, cv::Size(320, 240));
         
         upper_roi = frame(upper_roi_r);

          cv::imshow("upper", upper_roi);

      if (!lines.empty()) {

            
            cameraManager.detectSign(upper_roi);
       
            if (cameraManager.detectedSign == CameraManager::TrafficSign::NO_SIGN) {
                std::cout << "NO SIGN DETECTED" << std::endl;
                  /*
                    cameraManager.detectLines(frame);

                    if (cameraManager.direction == CameraManager::Direction::GO_FORWARD){
                        motorLock.lock();
                        mController.goForward(40);
                        delay(10);
                        motorLock.unlock();
                    }else if (cameraManager.direction == CameraManager::Direction::TURN_RIGHT_LITTLE) {
                        motorLock.lock();
                        mController.turnRight(50, 10);
                        delay(40);
                        mController.goForward(40);
                        motorLock.unlock();
                    }else if (cameraManager.direction == CameraManager::Direction::TURN_LEFT_LITTLE) {
                        motorLock.lock();
                        mController.turnLeft(50, 10);
                        delay(40);
                        mController.goForward(40);
                        motorLock.unlock();
                    }else if (cameraManager.direction == CameraManager::Direction::TURN_RIGHT_SHARPER) {
                        motorLock.lock();
                        mController.smoothTurn(70, 10);
                        delay(40);
                        motorLock.unlock();
                    }else if (cameraManager.direction == CameraManager::Direction::TURN_LEFT_SHARPER) {
                        motorLock.lock();
                        mController.smoothTurn(10, 70);
                        delay(40);
                        motorLock.unlock();
                    }*/
                        motorLock.lock();
                        mController.goForward(40);
                        delay(10);
                        motorLock.unlock();

            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::TRAFFIC_LIGHT_RED) {
                std::cout << "TRAFFIC LIGHT RED DETECTED" << std::endl;
                motorLock.lock();                
                mController.stopMotor();
                motorLock.unlock();    
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::TRAFFIC_LIGHT_GREEN) {
                std::cout << "TRAFFIC LIGHT GREEN DETECTED" << std::endl;
                motorLock.lock();
                mController.goForward(40);
                delay(40);
                motorLock.unlock();
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::LEFT_SIGN) {
                std::cout << "LEFT DETECTED" << std::endl;
                motorLock.lock();
                delay(200);
                mController.turnLeft(80, 10);
                delay(400);
                motorLock.unlock();
                
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::PARKING) {
                std::cout << "PARKING DETECTED" << std::endl;
                motorLock.lock();
                delay(800);
                mController.stopMotor();
                delay(60*1000);
                motorLock.unlock();
                capture.release();
                break;
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::STOP_SIGN) {
                std::cout << "STOP DETECTED" << std::endl;
                motorLock.lock();
                mController.stopMotor();
                delay(5000);
                mController.goForward(40);
                motorLock.unlock();
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::RIGHT_SIGN) {
                std::cout << "RIGHT DETECTED" << std::endl;
                motorLock.lock();
                 mController.turnRight(60, 10);
                 motorLock.unlock();
            }else if (cameraManager.detectedSign == CameraManager::TrafficSign::PEDESTRIAN) {
                std::cout << "PEDESTRAIN DETECTED" << std::endl;
                motorLock.lock();
                mController.turnRight(50, 90);
                delay(100);
                mController.goForward(20);
                delay(2000);
                isPedestrianDetected = true;
                motorLock.unlock();
            }
            
        cv::waitKey(25); 
    }
}
    
    irThread.join();
    ultraSonicThread.join();
    return 0;
}


void irObject() {
    int LValue, RValue; 

    while (true) {
        
        LValue = digitalRead(LEFT_IR_PIN);
        RValue = digitalRead(RIGHT_IR_PIN);

        if(LValue == 0 && RValue == 0 && objectCounter == 0) {
            std::cout << "OBSTACLE 1"<< std::endl;
            motorLock.lock();                          
            mController.stopMotor();
            delay(6000);
            mController.goForward(40);
            motorLock.unlock();
            objectCounter++;
        
        }else if (LValue == 0 && RValue == 0 && objectCounter == 1) { 
           
                std::cout << "OBSTACLE 2"<< std::endl;
                
                motorLock.lock();
                mController.turnRight(80, 10);
                delay(400);

                mController.goForward(40); 
                delay(500);
        
                mController.turnLeft(80, 0); 
                delay(400);
                
                mController.goForward(40); 
                delay(1800);
                
                mController.turnLeft(80, 0); 
                delay(500);
                
                mController.goForward(40); 
                delay(600);
                
                mController.turnRight(80, 10);
                delay(500);

                mController.goForward(40); 
                motorLock.unlock();
                objectCounter++;
        
        }
    }
}

    
void lineTracer() {	
	 
    
	   int leftSideLineTracer, rightSideLineTracer;
	  
     while (true) {
		 
		  leftSideLineTracer = lineTracerManager.readLeftLineTracerValue();
		  rightSideLineTracer = lineTracerManager.readRightLineTracerValue();
   
        
         if (leftSideLineTracer == 0 && rightSideLineTracer == 1) {
                 // turn right 
                 
                    motorLock.lock();
                    mController.turnRight(50,90);
                    delay(40);
                    mController.goForward(40);
                    motorLock.unlock();
                    
            
         }else if (rightSideLineTracer ==0 && leftSideLineTracer == 1) {
                  // turn left
                    motorLock.lock();
                    mController.turnLeft(50, 90);
                    delay(40);
                    mController.goForward(40);
                    motorLock.unlock();
           
         }else if (rightSideLineTracer == 0 && leftSideLineTracer == 0) {
                  // Both detected
                    motorLock.lock();
                    mController.goForward(70);
                    delay(200);
                    motorLock.unlock();
                               
         }else  {
             
                    motorLock.lock();
                    mController.goForward(40);
                    motorLock.unlock();
         }     
	 }

}

void init() {
    //Wiring Pi Setup  
    if(wiringPiSetup() == -1)
      return 0;
    //Init Object detection IR
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);
    // Init Line Tracer
    lineTracerManager.initilizeLineTacer();

    //Init Motor
    mController.setPins();
    mController.initializeDcMotor();

}




//g++ final.cpp ./cpp/line_tracer.cpp ./cpp/motor.cpp ./cpp/camera.cpp -o smartCar -pthread -lwiringPi -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv`


