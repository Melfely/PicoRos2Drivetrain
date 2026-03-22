#include "Sensor.h"
#include <cmath>

#pragma region Distance

/// @brief Constructor for a distance Sensor
/// @param TriggerPin This is the pin that will be used for starting the cycle. Is PWM
/// @param EchoPin This is the pin that will be used to return the time it took. Is GPIO
Sensor::Distance::Distance(uint TriggerPin, uint EchoPin)
:
TriggerPin(TriggerPin, 12, 49999), EchoPin(EchoPin, false)
{
    this->TriggerPin.SetDuty((uint)(6));
    this->EchoPin.SetPulls(false, true);

    this->EchoPin.SetIRQ<&Distance::echoHandler>(this, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

    this->distance = std::nullopt;
    this->startTime = 0;
}

/// @brief Method used by the echoHandler_Callback method to handle the measuring calculating the distance
/// @param events Internal Stuff
void Sensor::Distance::echoHandler(uint32_t events) {
    if (this->EchoPin.GetState() ) {
        this->startTime = time_us_64();
        //printf("StartTime: %llu -> ", startTime); //Debug Print
    } else {
            uint64_t dT = time_us_64() - this->startTime;
            //printf(" dT: %llu \n", dT); //Debug Print
            if (dT < 100) {
                this->distance = 0;
            } else if (dT > 100 && dT < 38000) {
                this->distance = dT / 58.0f / 100.0f;
            } else {
                this->distance = std::nullopt;
            }
            
    }
}

/// @brief Returns the non-thread Safe Distance read by the sensor
/// @return this will return the distance in meters as a float, if a -1.0 then that is out of range
float Sensor::Distance::GetDistance() {
    if (this->distance != std::nullopt) {
            return *this->distance; 
    } else {
            return -1.0f;
    }

}

#pragma endregion
#pragma region MotorEncoder

Sensor::MotorEncoder::MotorEncoder(uint pinA, uint pinB)
: encoder_pin_a(pinA, false), encoder_pin_b(pinB, false)
{

    encoder_pin_a.SetPulls(false, false);
    encoder_pin_b.SetPulls(false, false);

    this->pin_a_val = encoder_pin_a.GetState();
    this->pin_b_val = encoder_pin_b.GetState();

    this->encoder_counts = 0;
    this->previous_counts = 0;

    this->motor_angular_velocity = 0.0f;

    this->previous_tick = time_us_64();

    this->encoder_pin_b.SetIRQ<&Sensor::MotorEncoder::PinBHandler>(this, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    this->encoder_pin_a.SetIRQ<&Sensor::MotorEncoder::PinAHandler>(this, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    

    //Make the timer negative, so the time is ALWAYS accurate regardless of callback execution time
    add_repeating_timer_ms(-1 * (1000 / TIMER_FREQUENCY), TimeoutCheck_Callback, this, &timer);
    
}

void Sensor::MotorEncoder::PinAHandler(uint32_t events) {
    uint64_t now = time_us_64();
    this->pin_a_val = encoder_pin_a.GetState();
    //This will subtract when pinA and pinB are equal, otherwise will add
    /*
    a = 0, b = 0 subtract
    a = 1, b = 0 add
    a = 0, b = 1 add
    a = 1, b = 1 subtract    
    */

    if (pin_a_val != pin_b_val) {
        encoder_counts += 1;
    } else {
        encoder_counts -= 1;
    }
    this->motor_angular_velocity =  CalculateVelocity(now);

    //printf("PinA\n"); //debug line

}
void Sensor::MotorEncoder::PinBHandler(uint32_t events){
    uint64_t now = time_us_64();
    this->pin_b_val = encoder_pin_b.GetState();
    //This will add when pinA and pinB are equal, otherwise will subtract
    /*
    a = 0, b = 0 add
    a = 1, b = 0 subtract
    a = 0, b = 1 subtract
    a = 1, b = 1 add
    */

    if (pin_a_val != pin_b_val) {
        encoder_counts -= 1;
    } else {
        encoder_counts += 1;
    }
    this->motor_angular_velocity =  CalculateVelocity(now);

    //printf("PinB\n" ); //debug line
}

/// @brief Time between encoder count based velocity check, this should handle low speed motors MUCH more accurately, while also handling high speed motors
/// @param now the time when the encoder count triggered
/// @return the speed based on how long it was since the LAST time this was called, (therefore the last time this was triggered)
float Sensor::MotorEncoder::CalculateVelocity(uint64_t now) {
    uint64_t dT = now - this->previous_tick;
    float angVel = RADIAN_PER_ENCODER_COUNT / dT;


    return (encoder_counts >= previous_counts) ? angVel : angVel * -1;
    this->previous_tick = now;
}

void Sensor::MotorEncoder::TimeoutCheck(){
    
    if (this->previous_tick >= this->TIMEOUT_DELAY) {
        this->motor_angular_velocity = 0.0f;
    }

}

bool Sensor::MotorEncoder::TimeoutCheck_Callback(struct repeating_timer *t){
    //Take the void pointer from the user data of the timer, cast it to an MotorEncoder Pointer, then call the measureVelocity Function on said MotorEncoder
    MotorEncoder* self = (MotorEncoder*)t->user_data;
    self->TimeoutCheck();
    return true;

}

#pragma endregion


#pragma region MPU6050

Sensor::MPU6050::MPU6050(uint scl, uint sda, uint i2c)
: sclID(scl), sdaID(sda), i2cID(i2c)
{
    i2c_init(&i2c0_inst, 100 * 4000); //Use 400KHz
    gpio_set_function(sdaID, GPIO_FUNC_I2C);
    gpio_set_function(sclID, GPIO_FUNC_I2C);
    gpio_pull_up(sdaID);
    gpio_pull_up(sclID);

    //Write to PWMR_MGMT_1 register addr
    uint8_t write[] = {0x6B, 0x00}; 
    i2c_write_blocking(&i2c0_inst, i2cID, write, 2, false);

    gyro_bias_x = 0.0;
    gyro_bias_y = 0.0;
    gyro_bias_z = 0.0;
    calibrate_gyro(1000);
    
}

void Sensor::MPU6050::calibrate_gyro(uint32_t samples) {
    float ang_x_deposite = 0.0;
    float ang_y_deposite = 0.0;
    float ang_z_deposite = 0.0;


    for(int i = 0; i <= samples; i++) {
        data data = read_data();

        ang_x_deposite += data.ang_vel_x;
        ang_y_deposite += data.ang_vel_y;
        ang_z_deposite += data.ang_vel_z;
        sleep_ms(5);
    }

    gyro_bias_x = ang_x_deposite / samples;
    gyro_bias_y = ang_y_deposite / samples;
    gyro_bias_z = ang_z_deposite / samples;
}

Sensor::MPU6050::data Sensor::MPU6050::read_data() {

    uint8_t readStart = 0x3B; //The spot read from
    uint8_t bytes[14] = {0}; //The buffer to read into
    
    i2c_write_blocking(&i2c0_inst, i2cID, &readStart, 1, true);
    i2c_read_blocking(&i2c0_inst, i2cID, bytes, 14, false);

    uint16_t words[7] = {0};

    for(int i = 0; i < 7; i++) {
        words[i] = (bytes[i *2 ] << 8) | (bytes[i * 2 + 1]);
    }

    dataCache.lin_acc_x = process_raw(words[0], 16384) * 9.80665; 
    dataCache.lin_acc_y = process_raw(words[1], 16384) * 9.80665; 
    dataCache.lin_acc_z = process_raw(words[2], 16384) * 9.80665; 

    dataCache.ang_vel_x = (process_raw(words[4], 131) * DEGREE_TO_RAD) - gyro_bias_x;
    dataCache.ang_vel_y = (process_raw(words[5], 131) * DEGREE_TO_RAD) - gyro_bias_y;
    dataCache.ang_vel_z = (process_raw(words[6], 131) * DEGREE_TO_RAD) - gyro_bias_z;
    
    return dataCache;
}

#pragma endregion