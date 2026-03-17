#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <cstring>
#include <codecvt>
#include <time.h>

#include <string>

#include <iostream>

#include "DriveTrain.h"
#include "PWM.h"
#include "Sensor.h"

constexpr uint64_t UARTDELAYTX = 20; //UART Tx delay in ms this will be 50hz

//Thanks internet, should parse the strings!
std::pair<float, float> parseString(const char* str) {
    char* end1;
    char* end2;

    // 1. Parse the first float
    float x = std::strtof(str, &end1);

    // If the pointer didn't move, no number was found
    if (str == end1) return {0.0f, 0.0f};

    // 2. Skip the comma and any whitespace
    char* ptr = end1;
    while (*ptr == ',' || *ptr == ' ' || *ptr == '\t') {
        ptr++;
    }

    // 3. Parse the second float from the new position
    float y = std::strtof(ptr, &end2);

    // If the pointer didn't move from ptr, the second number failed
    if (ptr == end2) return {0.0f, 0.0f};

    return {x, y};
}

int main()
{
    stdio_init_all();

    uint64_t lastSend = time_us_64();
    char buffer[100];
    int bufferIndex = 0;

    PWM::LED BlueLED(26);
    PWM::LED GreenLED(27);
    PWM::LED RedLED(28);

    Drivetrain::MotorInit RightMotorInit(14,15,13,12,11); //Motor Bank R
    Drivetrain::MotorInit LeftMotorInit(17,16,18,19,20); //Motor Bank L

    Sensor::MPU6050 IMU; //Use default addrs
    Sensor::MPU6050::data imu_data;
    Drivetrain::EncodedDualMotor Drive(LeftMotorInit, RightMotorInit);
    Drive.SetState(true);

    while (true) {

        imu_data = IMU.read_data();
        
        if ((time_us_64() - lastSend) >= (UARTDELAYTX  * 1000)) {
            float linVel = Drive.LinVelocity();
            float angVel = Drive.AngularVelocity();
            printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                linVel, 
                angVel, 
                imu_data.lin_acc_x, 
                imu_data.lin_acc_y, 
                imu_data.lin_acc_z,
                imu_data.ang_vel_x,
                imu_data.ang_vel_y,
                imu_data.ang_vel_z
            );
            lastSend = time_us_64();
            BlueLED.Toggle();
        }
        
        //Read a single character
        int c = stdio_getchar_timeout_us(0);
        
        //Make sure its a real character
        while (c != PICO_ERROR_TIMEOUT) {

            if(c == '\n' || c == '\r') {
                buffer[bufferIndex];

                auto [linVel, angVel] = parseString(buffer);
                Drive.LiveCommandMotors(linVel, angVel);
                RedLED.Toggle();
                bufferIndex = 0;
            } else if (bufferIndex < sizeof(buffer) - 1) {
                buffer[bufferIndex++] = (char)c;
            }

            c = stdio_getchar_timeout_us(0); //Check another character
        }
        GreenLED.ToggleEvery(1);
        sleep_us(1000);
    }   
}
