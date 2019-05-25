
#include "../headers/motor_lib.h"
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

#define DC_1_PIN 1
#define DC_2_PIN 4
#define DC_3_PIN 5
#define DC_4_PIN 6

void MotorController::initializeDcMotor() {

    softPwmCreate(DC_1_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(DC_2_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(DC_3_PIN, MIN_SPEED, MAX_SPEED);
    softPwmCreate(DC_4_PIN, MIN_SPEED, MAX_SPEED);

}


void MotorController::setPins() {
    pinMode(DC_1_PIN, SOFT_PWM_OUTPUT);
    pinMode(DC_2_PIN, SOFT_PWM_OUTPUT);
    pinMode(DC_3_PIN, SOFT_PWM_OUTPUT);
    pinMode(DC_4_PIN, SOFT_PWM_OUTPUT);
}

void MotorController::goForward(int speed) {
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    softPwmWrite(DC_1_PIN, speed);
    softPwmWrite(DC_2_PIN, MIN_SPEED);
    softPwmWrite(DC_3_PIN, speed);
    softPwmWrite(DC_4_PIN, MIN_SPEED);
}


void MotorController::goBack(int speed){
    softPwmWrite(DC_1_PIN, LOW);
    softPwmWrite(DC_2_PIN, speed);
    softPwmWrite(DC_3_PIN, LOW);
    softPwmWrite(DC_4_PIN, speed);
}

void MotorController::smoothTurn(int leftSide, int rightSide){
    softPwmWrite(DC_1_PIN, leftSide);
    softPwmWrite(DC_2_PIN, MIN_SPEED);
    softPwmWrite(DC_3_PIN, rightSide);
    softPwmWrite(DC_4_PIN, MIN_SPEED);
}

void MotorController::pointTurnRight(int speed) {

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

  softPwmWrite(DC_1_PIN, speed);
  softPwmWrite(DC_2_PIN, MIN_SPEED);
  softPwmWrite(DC_3_PIN, MIN_SPEED);
  softPwmWrite(DC_4_PIN, speed);
}

void MotorController::pointTurnLeft(int speed) {

  if (speed > MAX_SPEED)
    speed = MAX_SPEED;

  softPwmWrite(DC_1_PIN, MIN_SPEED);
  softPwmWrite(DC_2_PIN, speed);
  softPwmWrite(DC_3_PIN, speed);
  softPwmWrite(DC_4_PIN, MIN_SPEED);
}

void MotorController::turnRight(int speed, float degree) {
    // need to calculate degree

    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    softPwmWrite(DC_1_PIN, speed);
    softPwmWrite(DC_2_PIN, LOW);
    softPwmWrite(DC_3_PIN, LOW);
    softPwmWrite(DC_4_PIN, speed);
}

void MotorController::turnLeft(int speed, float degree) {
    // need to calculate degree

    if (speed > MAX_SPEED)
        speed = MAX_SPEED;

    softPwmWrite(DC_1_PIN, LOW);
    softPwmWrite(DC_2_PIN, speed);
    softPwmWrite(DC_3_PIN, speed);
    softPwmWrite(DC_4_PIN, LOW);
}


void MotorController::stopMotor() {
    std::cout << "STOP IS CALLED" << std::endl;
    softPwmWrite(DC_1_PIN, MIN_SPEED);
    softPwmWrite(DC_2_PIN,MIN_SPEED);
    softPwmWrite(DC_3_PIN,MIN_SPEED);
    softPwmWrite(DC_4_PIN,MIN_SPEED);

}
