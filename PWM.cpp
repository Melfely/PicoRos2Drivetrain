#include "PWM.h"

#pragma region LED
/// @brief Creates a PWMLED using a non-default constructr for PWMPIN
/// @param pin the GPIO Pin to use for PWMLED
PWM::LED::LED(uint pin) : PWM::PIN(pin, 1000, 65535) {

}

#pragma endregion 

#pragma region Motor

/// @brief Creates a PWM plus two GPIO Pin Controlled Motor
/// @param ENPin The pin that will be PWM
/// @param PHPin the pin that will be GPIO PHPin, this is the pin that will control direction
/// @param SleepPin  The pin that will be GPIO Pin2, this is the pin that controls the drive board
PWM::MOTOR::MOTOR(uint ENPin, uint PHPin, uint SleepPin) 
: PWM::PIN(ENPin, 5000, -1), 
PHPin(PHPin, true),
SleepPin(SleepPin, true)
{

}

/// @brief Will Spin the motor forward at a given speed percentage of max
/// @param speed the rate to spin the motor at, as a range of 0 to 1
void PWM::MOTOR::Forward(float speed) {
    assert(speed <= 1 && speed >= 0); //Make sure speed is between 0 and 1
    PHPin.SetState(true);
    SleepPin.SetState(true);
    this->SetDuty(speed);
}

/// @brief Will Spin the motor backward at a given speed percentage of max
/// @param speed the rate to spin the motor at, as a range of 0 to 1
void PWM::MOTOR::Backward(float speed) {
    assert(speed <= 1 && speed >= 0); //Make sure speed is set between 0 and 1
    PHPin.SetState(false);
    SleepPin.SetState(true);
    this->SetDuty(speed);
}

/// @brief Sets the state of the sleep pin
/// @param state sleep pin state
void PWM::MOTOR::SetSleep(bool state) {
    SleepPin.SetState(state);
}

#pragma endregion

#pragma region PIN
 
    /// @brief Will set and configure a PWM pin and Slice. Note that it will set the frequency information based on what you give it
    /// @brief this information is PER SLICE, not pin. So if you have two pins on the same slice, the most recent set one will take over the clock
    /// @param pin the GPIO Pin
    /// @param frequency The wanted freqnecy in hZ
    /// @param wrapCounter the wrap counter, used for precision and hZ max value. This is the value COUNTED up to, per cycle. Use -1 to auto generate best fit
PWM::PIN::PIN(uint pin, int frequency, int wrapCounter)
: GPIO::PIN(pin), FREQUENCY(frequency), WRAPCOUNTER(wrapCounter), currentDuty(0)
{
    gpio_set_function(pinID, GPIO_FUNC_PWM);
    
    pwm_config config = pwm_get_default_config();
    float divider;
    if (wrapCounter != -1) {
        divider = clock_get_hz(clk_sys) / (FREQUENCY * WRAPCOUNTER);
    } else {
        uint64_t wrapDiv =  clock_get_hz(clk_sys) / FREQUENCY;

        if (wrapDiv > 65536) {
            divider = wrapDiv / 65536.0f;
            WRAPCOUNTER = wrapDiv / divider; 
        } else {
            divider = 1;
            WRAPCOUNTER = wrapDiv;
        }
    }
     

    pwm_config_set_clkdiv(&config, divider);
    //Subtract one since the counter starts from  0
    pwm_config_set_wrap(&config, WRAPCOUNTER - 1);

    SLICE = pwm_gpio_to_slice_num(pinID);
    CHANNEL = pwm_gpio_to_channel(pinID);
    pwm_init(SLICE, &config, true);
    pwm_set_enabled(pinID, true);
}

    /// @brief Sets the duty of the to the exact given value.
    /// @param duty The duty value, this is capped by WRAPCOUNTER stored in the pin
void PWM::PIN::SetDuty(uint duty) 
{
    pwm_set_gpio_level(pinID, (duty < WRAPCOUNTER ? duty : WRAPCOUNTER));
    currentDuty = (duty < WRAPCOUNTER ? duty : WRAPCOUNTER) / WRAPCOUNTER;
}
    /// @brief Sets the duty to the float percentage
    /// @param duty any float between 0 and 1
void PWM::PIN::SetDuty(float duty) 
{
    assert( 0 <= duty && duty <= 1 && "Duty must be float between 0 and 1");
    pwm_set_gpio_level(pinID, duty * WRAPCOUNTER);
    currentDuty = duty;
}

    /// @brief Will fade up to the given max brightness in given amount of time.
    /// @param msecs Time to fade up in milliseconds
    /// @param peak  max brightness, from 0 to 1
    /// @param diffVar A value, that if changes will cause the loop to break early
void PWM::PIN::FadeUp(int msecs, float peak, volatile std::atomic<int>& diffVar) 
{
    float time = 0;
    auto localDiffVar = diffVar;
    while(time < peak * 100 && localDiffVar == diffVar) 
    {
        this->SetDuty(time / 100.0f);
        sleep_us((msecs * 1000) / 100);
        currentDuty = time / 100.0f;
        time = time + peak;
    }
}

    /// @brief Will fade down from given max, to 0 in given amount of time
    /// @param msecs time in milliseconds to fade
    /// @param peak max brightness, from 0 to 1
    /// @param diffVar A value, that if changes will cause the loop to break early
void PWM::PIN::FadeDown(int msecs, float peak, volatile std::atomic<int>& diffVar) 
{
    float time = peak * 100;
    auto localDiffVar = diffVar;
    while (time >= 0 && localDiffVar == diffVar)
    {
        this->SetDuty(time / 100);
        sleep_us((msecs * 1000) / 100);
        currentDuty = time / 100.0f;
        time = time - peak;
    }
}

    /// @brief Sets duty to 0
void PWM::PIN::Stop() 
{
    pwm_set_gpio_level(pinID, 0);
}

/// @brief Toggles the duty between 0 and 1
void PWM::PIN::Toggle() {
    if (currentDuty == 0) {
        currentDuty = 1;
        this->SetDuty(currentDuty);
    } else {
        currentDuty = 0;
        this->SetDuty(currentDuty);
    }
}

/// @brief Sets the duty to max or off based on given bool
/// @param IsOn a true sets to max, false sets to 0
void PWM::PIN::SetState(bool IsOn) {
    if (IsOn) {
        currentDuty = 1;
        this->SetDuty(currentDuty);
    } else {
        currentDuty = 0;
        this->SetDuty(currentDuty);
    }
}

/// @brief Get the current "On" or "Off" Value
/// @return returns a bool, true is any number not 0
bool PWM::PIN::GetState() {
    //Return 0 if 0, otherwise return 1
    return currentDuty == 0 ? 0 : 1; 
}

/// @brief Gets the current float range from 0 to 1 of the duty
/// @return 0 to 1, duty percentage
float PWM::PIN::GetDuty() {
    return currentDuty;
}

#pragma endregion 