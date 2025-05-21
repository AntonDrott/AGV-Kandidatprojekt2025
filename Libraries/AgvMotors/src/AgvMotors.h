#ifndef AgvMotors_h
#define AgvMotors_h

struct Motor
{
    short forward = -1;
    short backward = -1;
    //y = kx + m, adjusts for difference in motor drivers
    int m = 0;
    float k = 0;
    int offset = 0;
    String motorName = "";

};

struct AgvState
{
    int x;
    int y;
    int angle; //in degrees
    float currentSpeed;
    bool atTargetPosition = true;
    float v1 = 0;
    float v2 = 0;
};

struct Settings
{
    //System speed, affects movement and rotation speed.
    float speed = 1;
    //Motors will only run if true.
    bool on = false;
    bool autoAdjustAngle = true;
    bool adaptiveSpeed = true;
    int angleMargin = 1;
    float rotationalAdjustDampening = 15;
    bool externalControl = false;
    //Desired direction of AGV front.
    int targetRotation = 0; //Must be in degrees
    int targetX; //cm
    int targetY; //cm
    int pwmFreq = 20000;
    int pwmRes = 8;
    bool angleSnapping = false;
    int rotationalClamp = 30;
    int snappingAngle = 30; //in degrees
    Motor* motor_0 = nullptr;
};

class AgvMotors
{
    public:
        Motor motorLF;
        Motor motorLB;
        Motor motorRF;
        Motor motorRB;
        void begin();
        void setPosition(int x, int y);
        void setDirection(float angle);
        void setState(int angle, int x, int y);
        //Agv will move from current position to entered position
        void trackToTarget();
        void runMotorTest();
        void identifyMotors();
        void testMotor(Motor& m);
        void halt();
        void setMotor(Motor& motor, int velocity);
        bool moveComplete = true;
        Settings settings;
        AgvState agvState;
        int rotationalAdjust = 0;
        int dangle = 0;
        String msg = "";
        void setAll(int pwm);
        void nudge(int angle, int t);
        int sign(float value){return (value >= 0 ? 1 : -1);}
    private:
        float deg2Rad(float angle){ return angle*=PI/180;}
        float rad2Deg(float angle){ return angle*=180/PI;}
        int calibratedPwm(Motor& motor, int pwm);
        float operatingSpeed;
        void autoRangeAngle(float& angle);
        void calculateMotorRatio(float angle);
        static void movementEngine(void* parameter);
        void pwmRotate(int amount);
        //Sets motor movement, -velocity = backwards.
        void updateMotors();
        void regulateRotation();
        void forceRotate(int time, int dir);
        int toPwm(float value){ return round(value*operatingSpeed*255);}
        //feed info to movementEngine, will slightly shift PWMs to rotate
        


};

#endif