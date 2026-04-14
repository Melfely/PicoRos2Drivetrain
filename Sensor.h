#ifndef SENSOR_H
#define SENSOR_H


#include "GPIO.h"
#include "PWM.h"
#include <optional>
#include "hardware/i2c.h"
#include <math.h>

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

            float AngularVelocity(){ return this->motor_angular_velocity / GEAR_RATIO; }
            void ResetEncoderCount() {this->encoder_counts = 0;}

            volatile int encoder_counts;
            volatile int previous_counts;
            volatile uint64_t previous_tick;

            #pragma endregion
        protected:
            MotorEncoder() = delete;
            #pragma region Constants & Statics
                static constexpr float GEAR_RATIO = 47.0f;
                static constexpr float ENCODER_COUNTS_PER_REV = 48.0f; //Pulse Counts per revolution
                static constexpr float RADIAN_PER_ENCODER_COUNT = ((2 * M_PI) / ENCODER_COUNTS_PER_REV);
                static constexpr float ENCODER_COUNTS_PER_RADIAN = 1 / RADIAN_PER_ENCODER_COUNT;
                static constexpr float TIMER_FREQUENCY = 10'000;
                static constexpr uint32_t TIMEOUT_DELAY = 100; //Delay in ms until 0.0 velocity can be reported.  
                static constexpr float US_TO_S = 1.0f / 1'000'000.0f; //microseconds to seconds

                
            #pragma endregion
            #pragma region Fields
                GPIO::PIN encoder_pin_a;
                GPIO::PIN encoder_pin_b;

                volatile bool pin_a_val;
                volatile bool pin_b_val;

                /// @brief Angular Velocity of the motor
                volatile float motor_angular_velocity; 

                struct repeating_timer timer;
            #pragma endregion
            #pragma region Protected Methods

            void PinAHandler(uint32_t events);
            void PinBHandler(uint32_t events);

            float CalculateVelocity(uint64_t now);

            void TimeoutCheck();

            static bool TimeoutCheck_Callback(struct repeating_timer *t);

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

            float gyro_bias_x;
            float gyro_bias_y;
            float gyro_bias_z;

            void calibrate_gyro(uint32_t samples);

            data dataCache; 

            float DEGREE_TO_RAD = M_PI / 180;
            
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