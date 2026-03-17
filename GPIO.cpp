#include "GPIO.h"

#pragma region PIN

/// @brief Creats and Inits a GPIO Pin.
/// @param pin The GPIOPin number to use 
/// @param output The mode. true for output, false for input.
GPIO::PIN::PIN(uint pin, bool output = true)
: pinID(pin)
{
    gpio_set_irq_callback(&MasterCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
    //Init the pin for Led Control
    gpio_init(pinID);
    gpio_set_dir(pinID, output);
}

/// @brief Constructor only for PWM GPIO Inits
/// @param pin the pin for the PWM GPIO Init
GPIO::PIN::PIN(uint pin) : pinID(pin)
{
    gpio_set_irq_callback(&MasterCallback);
}

/// @brief Toggles the state of the pin, in output mode.
void GPIO::PIN::Toggle() { 
    gpio_xor_mask(1u << pinID);
}

/// @brief Sets the pin output to given state
/// @param state the state to set the pin too
void GPIO::PIN::SetState(bool state) {
    gpio_put(pinID, state);
}

/// @brief Returns the state of GPIOPIN
/// @return the boolean state of the pin
bool GPIO::PIN::GetState() {
    return gpio_get(pinID);
}

/// @brief Returns the cached pin value
/// @return the cached uint value
uint GPIO::PIN::GetPin() {
    return pinID;
}

/// @brief Swaps between on and off, with the given delay as the MINIMUM, they actual delay will be based on loop execution speed
/// @param seconds the intended minimum delay between blinks, a fast code should blink this fast.
void GPIO::PIN::ToggleEvery(float seconds) {

    this->timePassed += (float)(time_us_64() - this->timeAtLastCall_us) / 1000000.0f;
    
    timeAtLastCall_us = time_us_64();
    if (this->timePassed >= seconds) {
        this->Toggle();
        this->timePassed = 0;
    }
}

void GPIO::PIN::SetPulls(bool PullUp, bool PullDown){
    gpio_set_pulls(pinID, PullUp, PullDown);
}

/// @brief Declare the array that stores the Irq callbacks
GPIO::RawCallback GPIO::PIN::rawCallbacks[NUM_BANK0_GPIOS];
/// @brief Stores the objects that want to have something called
void* GPIO::PIN::callbackContexts[NUM_BANK0_GPIOS];

void GPIO::PIN::DisableIRQ() {
    //Disable all events and set the arrays to nullptr
    gpio_set_irq_enabled(pinID, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE | GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, false);
    callbackContexts[pinID] = nullptr;
    rawCallbacks[pinID] = nullptr;

}

void GPIO::PIN::MasterCallback(uint pin, uint32_t eventMask) {
    
    //Grab the auto generated function
    RawCallback func = rawCallbacks[pin];

    //Grab the void* pointer to the pin, the function will know how to dereference it
    void* ctx = callbackContexts[pin];

    //Make sure both exist
    if (func && ctx) {
        func(ctx, eventMask); //call the auto generated callback handler 
    }

}



#pragma endregion

#pragma region LED

/// @brief Creates a LED using non-default constructor for GPIOPIN
/// @param pin the GPIO pin to use for LED
GPIO::LED::LED(uint pin) : PIN(pin, true) {

}

#pragma endregion

#pragma region BUTTON 

/// @brief Creates a button using non-default constructor for GPIOPIN
/// @param pin the GPIO pin to use for BUTTON
/// @param IsPullUp if this is a PULL UP button, if not then it will be a PULL DOWN
GPIO::BUTTON::BUTTON(uint pin, bool IsPullUp) : PIN(pin, false) 
{
    if (IsPullUp) {
        gpio_pull_up(pinID);
    } else {
        gpio_pull_down(pinID);
    }
}

/// @brief Checks if the pin is in pull up mode
/// @return this will return true if in pull up mode. False if in pull down.
bool GPIO::BUTTON::IsPullUp() {
    return gpio_is_pulled_up(pinID);
}

/// @brief This method will return if button is pressed. Does properly handle PULL UP or PULL DOWN differences
/// @return 
bool GPIO::BUTTON::IsPressed() {
    if (IsPullUp()) {
        return !gpio_get(pinID);
    } else {
        return gpio_get(pinID);
    }
}

#pragma endregion