
#ifndef MOTOR_HEADER
#define MOTOR_HEADER

#define MAX_SPEED 80
#define MIN_SPEED 0

class MotorController {

public:

  
    void initializeDcMotor();

    void setPins();

    void goBack(int speed);

    void goForward(int speed);

    void turnLeft(int speed, float degree = 90);

    void turnRight(int speed, float degree = 90);
    
    void smoothTurn(int leftSide, int rightSide);

    void stopMotor();
    
    void pointTurnRight(int speed);
    
    void pointTurnLeft(int speed);

};

#endif
