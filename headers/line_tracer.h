#ifndef LINE_TRACER_HEADER 
#define LINE_TRACER_HEADER

#define LEFT_TRACER_PIN 10
#define RIGHT_TRACER_PIN 11


class LineTracerController {

public:

	void initilizeLineTacer();
	int readLeftLineTracerValue();
	int readRightLineTracerValue();   

};

#endif



