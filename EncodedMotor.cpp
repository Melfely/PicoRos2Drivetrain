#include "EncodedMotor.h"
#include "DriveTrain.h" //Include this here instead of .h so no circular dependency issue
#include <algorithm>

#pragma region EncodedMotor


PWM::EncodedMotor* PWM::EncodedMotor::motorObjects[MAXMOTORS] = {};
bool PWM::EncodedMotor::isTimerInit = false;
struct repeating_timer PWM::EncodedMotor::timer;


PWM::EncodedMotor::EncodedMotor(Drivetrain::MotorInit& motorInit)
: 
MOTOR(motorInit.ENPin, motorInit.PHPin, motorInit.SleepPin),
MotorEncoder(motorInit.encPin1,motorInit.encPin2)
{
    //Create a timer that will trigger every millisecond regardless.
    if (!isTimerInit) {
        add_repeating_timer_us(-1 * (1000000 / TIMERFREQUENCY), HandleMotor_Callback, nullptr, &timer);
        isTimerInit = true;
    }
    
    bool slotFound = false;

    for (int i = 0; i < MAXMOTORS; i++ ) {
        if(motorObjects[i] == nullptr) {
            motorID = i;
            motorObjects[motorID] = this; //Add the instance of the motor to the list
            slotFound = true;
            break;
        }
    }


    if (!slotFound) panic("Too many motors, no free slots!");

}


/// @brief Will Spin the motor forward at a given speed either as a percetnage of max or as a raw meters / second value.
/// @param speed the rate to spin the motor at, as a range of 0 to 1 If in PID control mode at the given m/s if in PID control
void PWM::EncodedMotor::Forward(float speed) {
    
    this->SetSpeed(std::abs(speed));
    
}

/// @brief Will Spin the motor backward at a given speed either as a percetnage of max or as a raw meters / second value.
/// @param speed the rate to spin the motor at, as a range of 0 to 1 If in PID control mode at the given m/s if in PID control
void PWM::EncodedMotor::Backward(float speed) {
    
    this->SetSpeed(-1.0 * std::abs(speed));
    
}

/// @brief Sets the motor driver pins for going forward.
void PWM::EncodedMotor::Forward() {
    
    PHPin.SetState(true);
    //SleepPin.SetState(true);
}

/// @brief Sets the motor driver pins for going backwards. 
void PWM::EncodedMotor::Backward() {
    
    PHPin.SetState(false);
    //SleepPin.SetState(true);
}

/// @brief 
/// @param t 
/// @return 
bool PWM::EncodedMotor::HandleMotor_Callback(struct repeating_timer *t) {
    for(int i = 0; i < MAXMOTORS; i++){
        if (motorObjects[i] != nullptr) motorObjects[i]->HandleMotor(); //Call the motor only if one actually exists (slightly important)
    }
    return true;
}

// Save current optimization settings
//#pragma GCC push_options
// Force optimization to level 0 (None) for this section
//#pragma GCC optimize ("O0")

/// @brief Runs the pid, and auto stopping for a rotate counts command
void PWM::EncodedMotor::HandleMotor() {
    timerCounts++;
    
    if (timerCounts >= PIDRATE) { //Every pidRate calls of this timer, we should do the PID loop.
        uint64_t startTime = time_us_64();
        //If no command in a while, 
        if (lastCommandCounts >= PIDRATE ) {
            this->Stop();
        } 

        //If fresh command, clear all old data.
        if (abs(prevTargetSpeed - pidTargetSpeed) > INTEGRAL_TIMEOUT_SPEED_DIFF) {
            integralSum = 0.0f;
            prevError = 0.0f;
            prevTargetSpeed = pidTargetSpeed;
        }
        this->lastCommandCounts += 1;

        
        this->timerCounts = 0;

        //The Difference between the current set speed and the max set speed.
        float speedDelta = pidTargetSpeed - pidCurrentSpeed;

        //If we are deacclerating this will be 1.0, and 0.0 if accelerating.
        float isBreaking = (float)((pidCurrentSpeed * speedDelta) < 0.0f);

        //Branchless if statement. 
        float currentLimit = MAXACCEL + (MAXDEACCEL - MAXACCEL) * isBreaking;

        //How much the speed can change by this tick
        float velocityChange = std::clamp(speedDelta, -currentLimit, currentLimit);

        //The adjustedSpeedValue
        float adjustedCurrentSpeed = pidCurrentSpeed + velocityChange;
        adjustedCurrentSpeed = std::clamp(adjustedCurrentSpeed, -MAXSPEED, MAXSPEED);

        //The max allowed difference from the realVelocity
        float minAllowed = this->AngularVelocity() - MAXLAG;
        float maxAllowed = this->AngularVelocity() + MAXLAG;

        //Clamp the speed adjustment to the ranges
        adjustedCurrentSpeed = std::clamp(adjustedCurrentSpeed, minAllowed, maxAllowed);

        pidCurrentSpeed = adjustedCurrentSpeed;

        //Get us the speed error
        float error = pidCurrentSpeed - this->AngularVelocity();

        //Non-branching execution statement
        this->integralSum = integralSum * !(prevError == 0 && error == 0 && pidCurrentSpeed == 0);

        //Proprotional term
        float P = KP * error;

        //Intgeral term
        integralSum += error * DT;


        //Anti-Windup system, clamps the integral so it doesn't get stupid high
        float integralMax = maxOutput / (KI > 0 ? KI : 1.0f);
        integralSum = std::clamp(integralSum, -integralMax, integralMax);

        float I = KI * integralSum;

        //Derivtative Term
        float derivative = (error - prevError) * INV_DT;
        float D = KD * derivative;

        prevError = error;

        //Feedforward
        float F = KF * pidCurrentSpeed;

        float output = P + I + D + F;

        if (pidTargetSpeed == 0 || output < MINOUTPUT && output > -MINOUTPUT) {

            if (pidTargetSpeed == 0) {
                output = 0;
                this->Stop();
                SleepPin.SetState(false);
            } else {
                (output >= 0) ? output = MINOUTPUT : output = -MINOUTPUT;
                 
                SleepPin.SetState(sleepOverride);
            }
            
            
        } else {
            SleepPin.SetState(sleepOverride);
        }

        output = std::clamp(output, -maxOutput, maxOutput);

        //Set Direction and Speed

        this->SetDuty(std::abs(output));

        (output >= 0) ? Forward() : Backward(); 

        executionTime = time_us_64() - startTime;
    }
}

//Renable standard optimizations
//#pragma GCC pop_options

/// @brief Set the target PID speed, and or speedMag
/// @param speed the value in rad/s
void PWM::EncodedMotor::SetSpeed(float speed) {

    float targetRadS = speed;
    lastCommandCounts = 0;

    targetRadS = std::clamp(targetRadS, -MAXSPEED, MAXSPEED);

    this->speedMag = std::abs(targetRadS);
    this->pidTargetSpeed = targetRadS;
    
}

/// @brief Stop the motor, and clearing any PID information
void PWM::EncodedMotor::Stop() {
    this->pidTargetSpeed = 0;

    lastCommandCounts = 0;

}

/// @brief Remove the motor from the call list, and remove one motor from the count
PWM::EncodedMotor::~EncodedMotor() {
    motorObjects[motorID] = nullptr;
}

#pragma endregion