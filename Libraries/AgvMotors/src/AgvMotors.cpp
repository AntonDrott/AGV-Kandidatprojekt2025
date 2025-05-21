#include "Arduino.h"
#include "AgvMotors.h"

const float deg2rad = PI/180;
const float rad2deg = 180/PI;

void AgvMotors::begin()
{

    motorLF.motorName = "Front Left";
    motorLB.motorName = "Back Left";
    motorRF.motorName = "Front Right";
    motorRF.motorName = "Back Right";

    operatingSpeed = settings.speed;

    pinMode(motorLF.forward, OUTPUT);
    pinMode(motorLB.forward, OUTPUT);
    pinMode(motorRF.forward, OUTPUT);
    pinMode(motorRB.forward, OUTPUT);

    pinMode(motorLF.backward, OUTPUT);
    pinMode(motorLB.backward, OUTPUT);
    pinMode(motorRF.backward, OUTPUT);
    pinMode(motorRB.backward, OUTPUT);

    ledcAttach(motorLF.forward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorLB.forward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorRF.forward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorRB.forward, settings.pwmFreq, settings.pwmRes);

    ledcAttach(motorLF.backward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorLB.backward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorRF.backward, settings.pwmFreq, settings.pwmRes);
    ledcAttach(motorRB.backward, settings.pwmFreq, settings.pwmRes);

    xTaskCreatePinnedToCore( //#SoundCloud rapper name, lil' task X
        AgvMotors::movementEngine,
        "MovementEngine",
        2048, //should be enough mem
        this, //send this into task params
        1,
        nullptr,
        1
    );
    
}

int AgvMotors::calibratedPwm(Motor& motor, int pwm)
{
    int pwm0 = pwm;
    pwm0 += settings.motor_0->offset;
    if(&motor == settings.motor_0){return(pwm0);}
    float rpm0 =  pwm0*settings.motor_0->k + settings.motor_0->m;
    return round((rpm0-motor.m) / motor.k);
}

void AgvMotors::nudge(int angle, int t)
{
    calculateMotorRatio(angle);
    updateMotors();
    delay(t);
    halt();

}

void AgvMotors::setState(int angle, int x, int y)
{
    agvState.angle = angle; //Must be in radians, things go haywire otherwise
    agvState.x = x;
    agvState.y = y;
}

void AgvMotors::setMotor(Motor& motor, int velocity)
{
    int pwm = calibratedPwm(motor,abs(velocity));
    msg = "Setting motor to:" + String(pwm) + " for " + motor.motorName;
    
    if(velocity < 0)
    {
        ledcWrite(motor.backward, abs(pwm));
        ledcWrite(motor.forward, 0);
    }
    else if (velocity != 0)
    {
        ledcWrite(motor.forward, abs(pwm));
        ledcWrite(motor.backward, 0);
    }
    else
    {
        ledcWrite(motor.backward, 0);
        ledcWrite(motor.forward, 0);
    }
    
}

void AgvMotors::identifyMotors()
{
    testMotor(motorLF);
    testMotor(motorRF);
    testMotor(motorRB);
    testMotor(motorLB);
    halt();
}



void AgvMotors::testMotor(Motor& m)
{
    setMotor(m, 250);
    delay(1000);
    setMotor(m,-250);
    delay(1000);
    setMotor(m,0);
    delay(1000);
}

void AgvMotors::runMotorTest()
{
    settings.on = true;
    settings.speed = 0.25; //slow run
    forceRotate(2000, 1);
    settings.speed = 0.75;
    forceRotate(1000,-1);
    delay(500);
    settings.speed = 0.50;
    calculateMotorRatio(0);
    updateMotors();
    delay(500);
    calculateMotorRatio(180*deg2rad);
    updateMotors();
    delay(500);
    calculateMotorRatio(90*deg2rad);
    updateMotors();
    delay(500);
    calculateMotorRatio(270*deg2rad);
    updateMotors();
    delay(500);
    halt();

}

void AgvMotors::trackToTarget() //TargetRotation in degrees
{
    int dx = settings.targetX - (agvState.x);
    int dy = settings.targetY - (agvState.y);

    if(settings.adaptiveSpeed)
    {
        float dist = sqrt(pow(dx,2) + pow(dy,2));

        if(dist <= 20)
        {
            operatingSpeed = 0.1;
        }
        else if(dist <= 50)
        {
            operatingSpeed = 0.2;
        }
        else
        {
            operatingSpeed = settings.speed;
        }

    }

    //int ddist = sqrt(pow(dx,2) + pow(dy,2) );
    float targetAngle = atan2(dy,dx) -((settings.targetRotation)*deg2rad);
    float angleError = targetAngle - (agvState.angle);
    autoRangeAngle(angleError);
    if(settings.angleSnapping)
    {
        //Wrap angle around one turn
        targetAngle = fmod((fmod(targetAngle , 2*PI) + 2*PI) , 2*PI);
        //This snaps the angle to the closest degree in the range of
        targetAngle = round(targetAngle/(settings.snappingAngle*deg2rad))*settings.snappingAngle*deg2rad;
    }
    calculateMotorRatio(targetAngle);
}

//this one is from mr GPT. Makes sure angle is within 0-2pi range
void AgvMotors::autoRangeAngle(float& angle)
{
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0) angle += 2.0 * PI;
}

void AgvMotors::halt()
{
    setMotor(motorLF, 0);
    setMotor(motorLB, 0);
    setMotor(motorRF, 0);
    setMotor(motorRB, 0);
    rotationalAdjust = 0;
}

void AgvMotors::calculateMotorRatio(float angle)
{
   agvState.v1 = cos(angle) - sin(angle);
   agvState.v2 = cos(angle) + sin(angle);

    float mag = max(fabs(agvState.v1), fabs(agvState.v2)); //sets mag to whichever value is bigger
    agvState.v1 /= mag; //Normalize to that value, this normalizes the speed at every angle.
    agvState.v2 /= mag;

}

void AgvMotors::forceRotate(int time, int dir)
{

    setMotor(motorLF, 255*settings.speed*dir);
    setMotor(motorLB, 255*settings.speed*dir);

    setMotor(motorRF, -255*settings.speed*dir);
    setMotor(motorRB, -255*settings.speed*dir);

    delay(time);

    halt();
}

void AgvMotors::updateMotors()
{
    setMotor(motorLF, toPwm(agvState.v1)-rotationalAdjust);
    setMotor(motorLB, toPwm(agvState.v2)-rotationalAdjust);
    setMotor(motorRF, toPwm(agvState.v2)+rotationalAdjust);
    setMotor(motorRB, toPwm(agvState.v1)+rotationalAdjust);
}


void AgvMotors::setAll(int pwm)
{
    setMotor(motorLF, pwm);
    setMotor(motorLB, pwm);
    setMotor(motorRF, pwm);
    setMotor(motorRB, pwm);
}


void AgvMotors::regulateRotation()
{
    dangle = int(settings.targetRotation - agvState.angle + 540) % 360 - 180;
    //rotationalAdjust = std::clamp(round(float(dangle/settings.rotationalAdjustDampening)),-settings.rotationalClamp, settings.rotationalClamp);
    rotationalAdjust = round(dangle/settings.rotationalAdjustDampening);
    if(abs(dangle) < settings.angleMargin){rotationalAdjust = 0;}
}



void AgvMotors::movementEngine(void* parameter)
{
    AgvMotors* agv = static_cast<AgvMotors*>(parameter);
    
    while(1)
    {
        if(!agv->settings.on) 
        {
            agv->halt();
            vTaskDelay(pdMS_TO_TICKS(200)); 
        } //if off, check again in 200ms
        else
        {
            if(!agv->settings.adaptiveSpeed)
            {
                agv->operatingSpeed = agv->settings.speed;
            }
            if(agv->settings.autoAdjustAngle)
             {
                agv->regulateRotation();
                if(!agv->settings.externalControl && agv->rotationalAdjust != 0)
                {
                    agv->setMotor(agv->motorLF, -(agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorLF.offset));
                    agv->setMotor(agv->motorLB, -(agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorLB.offset));
                    agv->setMotor(agv->motorRF, agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorRF.offset);
                    agv->setMotor(agv->motorRB, agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorRB.offset);
                }
                else if(!agv->settings.externalControl)
                {
                    agv->halt();
                }
            }
            else
            {
                agv->rotationalAdjust = 0;
            }
                //if(!agv->agvState.atTargetPosition) {agv->trackToTarget(agv->settings.targetX, agv->settings.targetY);}
            if(agv->settings.externalControl)
            {
                if(abs(agv->dangle) <= 4)
                {
                    agv->trackToTarget();
                    agv->updateMotors();
                }
                else
                {
                    agv->setMotor(agv->motorLF, -(agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorLF.offset));
                    agv->setMotor(agv->motorLB, -(agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorLB.offset));
                    agv->setMotor(agv->motorRF, agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorRF.offset);
                    agv->setMotor(agv->motorRB, agv->rotationalAdjust + agv->sign(agv->rotationalAdjust)*agv->motorRB.offset);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

}