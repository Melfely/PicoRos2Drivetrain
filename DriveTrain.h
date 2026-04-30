#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "PWM.h"
#include "GPIO.h"
#include "Sensor.h"
#include "EncodedMotor.h"

namespace Drivetrain
{
    
    struct MotorInit {
        public: 
            MotorInit(uint ENPin, uint PHPin, uint SleepPin, uint encPin1, uint encPin2);
            MotorInit(uint ENPin, uint PHPin, uint SleepPin);

            uint ENPin;
            uint PHPin;
            uint SleepPin;
            uint encPin1;
            uint encPin2;
        protected:
            MotorInit() = delete;
    };

    class DualMotor {
        public:
            DualMotor(MotorInit LeftMotorInit, MotorInit RightMotorInit);

            virtual ~DualMotor();

            virtual void Forward(float speed);
            virtual void Backward(float speed);

            virtual void SpinLeft(float speed);
            virtual void SpinRight(float speed);

            virtual void SetState(bool state);

            virtual void Stop();

            virtual float GetLeftDuty();
            virtual float GetRightDuty();

        protected:
            PWM::MOTOR* LeftMotor;
            PWM::MOTOR* RightMotor;

            DualMotor();

    };

    #pragma region EncodedDualMotor
    class EncodedDualMotor : DualMotor {
        public:
            EncodedDualMotor(MotorInit LeftMotorInit, MotorInit RightMotorInit);

            void LiveCommandMotors(float linVel, float angVel);

            float LinVelocity();

            float AngularVelocity();

            using Drivetrain::DualMotor::Forward;
            using Drivetrain::DualMotor::Backward;
            using Drivetrain::DualMotor::SpinLeft;
            using Drivetrain::DualMotor::SpinRight;

            using Drivetrain::DualMotor::SetState;
            using Drivetrain::DualMotor::Stop;

            using Drivetrain::DualMotor::GetLeftDuty;
            using Drivetrain::DualMotor::GetRightDuty;

            virtual PWM::EncodedMotor* _RightMotor(){return static_cast<PWM::EncodedMotor*>(RightMotor);};
            virtual PWM::EncodedMotor* _LeftMotor(){return static_cast<PWM::EncodedMotor*>(LeftMotor);};

        protected:

            #pragma region Variables


            #pragma endregion
            #pragma region Statics and Constants

            static constexpr float WHEELBASE = 0.215f;
            static constexpr float INV_WHEELBASE = 1 / WHEELBASE;
            static constexpr float WHEELRADIUS = 0.033f;
            static constexpr float INV_WHEELRADIUS = 1 / WHEELRADIUS;
            static constexpr float ANG_FACTOR = 0.5f * WHEELBASE;


            static constexpr float MIN_LIN_SPEED = 0.10f; //The minimum linear speed
            static constexpr float MIN_ANG_SPEED = 0.60f; //The minimum angular speed
            static constexpr float MIN_ANG_COMPONENT = MIN_ANG_SPEED * ANG_FACTOR; //The minimum angular component
            static constexpr float MIN_TARGET_SPEED = MIN_LIN_SPEED - (0.5f * MIN_ANG_SPEED * WHEELBASE); //Calculate the min target speed
            #pragma endregion

        private:
            EncodedDualMotor() = delete;
    };
    #pragma endregion
} // namespace DualMotor

#endif