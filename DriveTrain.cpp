#include "DriveTrain.h"
#include <cmath>
#include <algorithm>

#pragma region DualMotor

/// @brief Inits a two motor drive, using two PWM::MOTORS, assumes a motor drive that utilizes a STBYpin
/// @param STBYPin The Pin the STBY system will run on, this allows this class to enable and disable motor output
/// @param LeftMotorInit the init struct that will store all the needed values for init the left motor. This does NOT use encoders
/// @param RightMotorInit the init struct that will store all the needed values for init on the right motor. This does NOT use encoders
Drivetrain::DualMotor::DualMotor(MotorInit LeftMotorInit, MotorInit RightMotorInit) 
:
LeftMotor( new PWM::MOTOR(LeftMotorInit.ENPin, LeftMotorInit.PHPin, LeftMotorInit.SleepPin)),
RightMotor(new PWM::MOTOR(RightMotorInit.ENPin, RightMotorInit.PHPin, RightMotorInit.SleepPin))
{

}

Drivetrain::DualMotor::DualMotor() 
:
LeftMotor(nullptr),
RightMotor(nullptr)
{

}

/// @brief Classic ol deconstructor. Cleans up any pointers;
Drivetrain::DualMotor::~DualMotor() {
    delete LeftMotor;
    delete RightMotor;
}



/// @brief Stops the motors
void Drivetrain::DualMotor::Stop() {
    LeftMotor->Stop();
    RightMotor->Stop();
}

/// @brief sets both motors to the same forward duty speed
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::Forward(float speed) {
    LeftMotor->Forward(speed);
    RightMotor->Forward(speed);
}

/// @brief sets both motors to the same forward duty speed
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::Backward(float speed) {
    LeftMotor->Backward(speed);
    RightMotor->Backward(speed);
}

/// @brief Sets the right motor to spin forward at given speed, and left motor to spin backwards at given speed, will be a left spin
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::SpinLeft(float speed) {
    LeftMotor->Backward(speed);
    RightMotor->Forward(speed);
}

/// @brief Sets the right motor to spin backward at given speed, and left motor to spin forward at given speed, will be a right spin
/// @param speed a number between 0 and 1 that is the wanted speed
void Drivetrain::DualMotor::SpinRight(float speed){ 
    LeftMotor->Forward(speed);
    RightMotor->Backward(speed);
}

/// @brief Sets the Sleep pins of both motors to the given state
/// @param state state to sleep to
void Drivetrain::DualMotor::SetState(bool state) {
    LeftMotor->SetSleep(state);
    RightMotor->SetSleep(state);
}

/// @brief Will return the duty or speed of the left motor
/// @return returns a float from 0 to 1
float Drivetrain::DualMotor::GetLeftDuty() {
    return LeftMotor->GetDuty();
}

/// @brief Will return the duty or speed of the left motor
/// @return returns a float from 0 to 1
float Drivetrain::DualMotor::GetRightDuty() {
    return RightMotor->GetDuty();
}

#pragma endregion

#pragma region EncodedDualMotor

Drivetrain::EncodedDualMotor::EncodedDualMotor(MotorInit LeftMotorInit, MotorInit RightMotorInit)
:
DualMotor()
{
    LeftMotor = new PWM::EncodedMotor(LeftMotorInit);
    RightMotor = new PWM::EncodedMotor(RightMotorInit);
}

/// @brief Commands the movement of the robot
/// @param linVel m/s command linear
/// @param angVel the angular velocity commandaded in rad/s
void Drivetrain::EncodedDualMotor::LiveCommandMotors(float linVel, float angVel) {
    
    //Calculate the needed turning component of the wheel command
    float ang_component = angVel * ANG_FACTOR;

    //Convert these to abs
    float abs_linVel = abs(linVel);
    float abs_ang_component = abs(ang_component); 

    //If we are below the min speed commands we will increase the commanded values by an amount to keep the relation the same
    //Just increases the command by enough to hopefully move.
    if (abs_linVel < MIN_LIN_SPEED && abs_ang_component < MIN_ANG_COMPONENT) {
        //Only work if Greater than a noise floor value
        const float NOISE_FLOOR = 0.001f;
        if(abs_linVel > NOISE_FLOOR || abs_ang_component > NOISE_FLOOR) {
            //Check to make sure the command is above 0, because if not we need to not divide by it.
            float lin_scale = (abs_linVel > 0.0f) ? (MIN_LIN_SPEED / abs_linVel) : 0.0;
            float ang_scale = (abs_ang_component > 0.0f) ? (MIN_ANG_COMPONENT / abs_ang_component) : 0.0f; 

            //Scale by the smaller of the two factors unless one is zero, then scale by the non-zeo
            float scale = (lin_scale != 0.0f && ang_scale != 0.0f) ? 
            std::min(lin_scale, ang_scale) : std::max(lin_scale, ang_scale); 
            
            linVel *= scale;
            ang_component *= scale;
            
        }
    }

    //Calculate the side specific output values. 
    float leftTargetVel = linVel - ang_component;
    float rightTargetVel = linVel + ang_component;

    //Send them to the motors and convert to radians per second
    _LeftMotor()->SetSpeed(leftTargetVel * INV_WHEELRADIUS);
    _RightMotor()->SetSpeed(rightTargetVel * INV_WHEELRADIUS);

}


float Drivetrain::EncodedDualMotor::LinVelocity() {
    float leftVel = _LeftMotor()->AngularVelocity() * WHEELRADIUS;
    float rightVel = _RightMotor()->AngularVelocity() * WHEELRADIUS;

    return 0.5 * (leftVel + rightVel);
}

float Drivetrain::EncodedDualMotor::AngularVelocity() {
    float leftVel = _LeftMotor()->AngularVelocity() * WHEELRADIUS;
    float rightVel = _RightMotor()->AngularVelocity() * WHEELRADIUS;

    return INV_WHEELBASE * (rightVel - leftVel);
}
#pragma endregion


#pragma region MotorInit

Drivetrain::MotorInit::MotorInit(uint ENPin, uint PHPin, uint SleepPin) {
    this->ENPin = ENPin;
    this->PHPin = PHPin;
    this->SleepPin = SleepPin;
}

Drivetrain::MotorInit::MotorInit(uint ENPin, uint PHPin, uint SleepPin, uint encPin1, uint encPin2) {
    this->ENPin = ENPin;
    this->PHPin = PHPin;
    this->SleepPin = SleepPin;
    this->encPin1 = encPin1;
    this->encPin2 = encPin2;
}


#pragma endregion