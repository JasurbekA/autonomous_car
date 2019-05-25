#include "../headers/line_tracer.h"
#include <wiringPi.h>
#define LEFT_TRACER_PIN 10
#define RIGHT_TRACER_PIN 11

void LineTracerController::initilizeLineTacer(){
	pinMode(LEFT_TRACER_PIN, INPUT);
	pinMode(RIGHT_TRACER_PIN, INPUT);		
}

int LineTracerController::readLeftLineTracerValue(){
	return digitalRead(LEFT_TRACER_PIN);
}
int LineTracerController::readRightLineTracerValue(){
	return digitalRead(RIGHT_TRACER_PIN);
}
