#ifndef ENCODEDMOTOR_H
#define ENCODEDMOTOR_H

#include "PWM.h"
#include "Sensor.h"
#include <cmath>

//Declare this exists somewhere
namespace Drivetrain {
    struct MotorInit;
}

namespace PWM 
{
    
    class EncodedMotor : public PWM::MOTOR, public Sensor::MotorEncoder {
        public: 
            EncodedMotor(Drivetrain::MotorInit&);

            virtual ~EncodedMotor();

            void Forward(float speed) override;

            void Backward(float speed) override;

            void SetSleep(bool state) override {sleepOverride = state;}

            void SetSpeed(float speed);

            void SetTargetSpeed(float newSpeed) {pidTargetSpeed = newSpeed; speedMag = std::abs(newSpeed);};
            
            virtual void Stop() override;

            float GetTargetSpeed() {return pidTargetSpeed;};
            float GetCurrentSetSpeed() {return pidCurrentSpeed;};
            float GetSpeedMag() {return speedMag;};

            static constexpr float MAXSPEED = 25.6f; //Max Speed Magnitude in Rad/s 
            
          
        protected:

            bool sleepOverride = false;

            uint64_t executionTime; //Test how long it takes for the pid to run, in microseconds
            
            static struct repeating_timer timer; //The timer for the PID and count system
            volatile int lastCommandCounts = 0; //The number of pid cylces since the last command, if this becomes equal to the value PIDRATE then the motor will call stop

            volatile float speedMag = 0; //The magnitude of the wanted current speed. Should NOT have a sign
            
            void Forward();

            void Backward();

            static bool HandleMotor_Callback(struct repeating_timer *t);
            virtual void HandleMotor();

            int timerCounts = 0; //The number of cycles since last PID check

            float pidCurrentSpeed = 0.0f; //The pid speed before the acceleration system, is signed.
            volatile float pidTargetSpeed = 0.0f; //The set speed of the PID
            float prevTargetSpeed = 0.0f;// The previously set target speed;

            float prevError = 0.0f; //The previous error value for the PID 
            float integralSum = 0.0f; //The integralCounter

            float maxOutput = 1.0f; //The max value of the PidController

            int motorID;

            static constexpr int PIDRATE = 100; //Number of timer cycles between each PID check
            static constexpr int TIMERFREQUENCY = 10000; //The number of times the timer is called per second. 
            static constexpr float DT = PIDRATE / (TIMERFREQUENCY * 1.0f); //The rawTime between each pidCheck
            static constexpr float INV_DT = 1 / DT;
            static constexpr float KF = 0.042; //Feedforward Constant
            static constexpr float KP = 0.075; //Proportional Constant 
            static constexpr float KI = 0.01; //Integral Constant 
            static constexpr float KD = 0.0; //Derivative Constant

            static constexpr float MINOUTPUT = 0.05; //Minimum commanded output 

            static constexpr float MAXACCEL = 100 * DT; //Max Acceleration
            static constexpr float MAXDEACCEL = 1000.0 * DT; //Max DeAcceleration or breaking

            static constexpr float MAXLAG = MAXSPEED * 0.25; //The max difference from commanded speed to real speed the acceleration can be ahead by.
            static constexpr int MAXMOTORS = 4;


            static PWM::EncodedMotor* motorObjects[MAXMOTORS]; //Max of 4 Motors, each motor takes 5 pins, using 20 pins for the motors leaves only 8 for other objects.
            static bool isTimerInit; //Is the timer for the motors init yet.
            
    };



}


#endif