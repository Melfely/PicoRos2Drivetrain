#ifndef SENSOR_H
#define SENSOR_H


#include "GPIO.h"
#include "PWM.h"
#include <optional>
#include "hardware/i2c.h"

namespace Sensor {

    class Distance {
        public:
            Distance(uint TriggerPin, uint EchoPin);
            float GetDistance();

        protected:
            /// @brief The PWM signal generator to allow the distance sensor to function.
            PWM::PIN TriggerPin;
            /// @brief The Echo or reciving pin to recieve the return signal
            GPIO::PIN EchoPin;

            Distance() = delete;

            void echoHandler(uint32_t events);

            std::optional<float> distance; //Distance to wall in meters
            uint64_t startTime;



    };
    #pragma region MotorEncoder
    class MotorEncoder {
        public:

            
            MotorEncoder(uint pinA, uint pinB);
            #pragma region Public Methods

            float AngularVelocity(){ return wheelAngVelocity;}
            void ResetEncoderCount() {this->encoderCounts = 0;}

            volatile int encoderCounts;
            volatile int previousCounts;

            #pragma endregion
        protected:
            MotorEncoder() = delete;
            #pragma region Constants & Statics
                static constexpr float gearRatio = 47;
                static constexpr float encoderCPR = 48; //Pulse Counts per revolution
                static constexpr float timerFrequency = 100;

            #pragma endregion
            #pragma region Fields
                GPIO::PIN EncodPinA;
                GPIO::PIN EncodPinB;

                volatile bool pinAVal;
                volatile bool pinBVal;


                /// @brief Angular Velocity of the wheel
                volatile float wheelAngVelocity;

                struct repeating_timer timer;
            #pragma endregion
            #pragma region Protected Methods

            void PinAHandler(uint32_t events);
            void PinBHandler(uint32_t events);

            void MeasureVelocity();

            static bool MeasureVelocity_Callback(struct repeating_timer *t);

            #pragma endregion
            

    };
    #pragma endregion

    class MPU6050 {
        public:

            struct data {
                float lin_acc_x = 0;
                float lin_acc_y = 0;
                float lin_acc_z = 0;
                
                float ang_vel_x = 0;
                float ang_vel_y = 0;
                float ang_vel_z = 0;
            };
            
            MPU6050(uint sclID=8, uint sdaID=9, uint i2cID=0x68);

            data read_data();


        protected:

            uint sclID;
            uint sdaID;
            uint i2cID;

            data dataCache; 
            
            /// @brief Converts from a raw word to a usable float
            /// @param val a uint that contains the (word) of sensor data
            /// @param scale a constant coefficient scaling raw data. accelerometer: 16384 per g | gyro: 131 per deg/s, TODO: use radians
            /// @return  human readible value in m/s^2 or deg/s
            float process_raw(uint16_t val, float scale) {
                if (val > 32768) {
                    return (val - 65535) / scale;
                }
                else {
                    return val / scale;
                }
            }

        
    };
}


#endif