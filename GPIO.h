//Helper methods to simplfy creating of many projects
#ifndef GPIO_H
#define GPIO_H


#include <stdio.h>
#include "pico/stdlib.h" 
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <cassert>
#include <atomic>
#include <functional>

namespace GPIO
{

    typedef void (*RawCallback)(void* instance, uint32_t events);
    class PIN {
        public:
            PIN(uint pin, bool output);

            virtual void Toggle();

            virtual void SetState(bool state);

            virtual bool GetState();

            virtual uint GetPin();

            virtual void ToggleEvery(float seconds);

            virtual void SetPulls(bool PullUp, bool PullDown);

            /// @brief Sets up the IRQ for this function
            /// @tparam T The objects type that will be calling this, can be skipped when using "this"
            /// @tparam MemberFunction the thing to callback
            /// @param instance the object to call 99% of the time will just be "this"
            /// @param eventMask when to trigger the IRQ
            template <auto MemberFunction, typename T>
            void SetIRQ(T* instance, uint32_t eventMask) {
                //Disable IRQ before setting anything, to prevent race conditions
                this->DisableIRQ();

                //assign the context to this object, its a void* it can hold anything
                //but the thing that uses it needs to know how to handle it
                callbackContexts[pinID] = instance;

                //Auto generate the callback to handle the void*
                rawCallbacks[pinID] = &Thunk<MemberFunction, T>;

                //Set the correct eventMask to true
                gpio_set_irq_enabled(pinID, eventMask, true);
            }

            void DisableIRQ();

            virtual ~PIN() {DisableIRQ();};

        protected:
            PIN(uint pin);
            PIN() = delete; //Remove default constructor
            const uint pinID;
            uint64_t timeAtLastCall_us= 0;
            float timePassed = 0;

            // Delete Copy Constructor (Prevent PIN b = a)
            PIN(const PIN&) = delete;

            // Delete Assignment Operator (Prevent b = a)
            PIN& operator=(const PIN&) = delete;

            

        private:
            static RawCallback rawCallbacks[NUM_BANK0_GPIOS];
            static void* callbackContexts[NUM_BANK0_GPIOS];


            static void MasterCallback(uint gpio, uint32_t events);

            /// @brief Generates a function that can handle dereference the IRQ handler for this code. It generates at COMPILE Time, so it is VERY fast
            /// @tparam T type that the void* will be casted from
            /// @tparam MemberFunction the function that will be called on the casted pointer
            /// @param instance the pointer to casted
            /// @param events the eventMask that triggered
            template<auto MemberFunction, typename T> static void Thunk(void* instance, uint32_t events) {
                T* self = static_cast<T*>(instance); //Cast the instance void* to the correct type

                (self->*MemberFunction)(events); //Call the correct method with the event that triggered it
            }

    };

    class LED : PIN {

        public:
            LED(uint pin);

            using PIN::Toggle;
            using PIN::SetState;
            using PIN::GetState;
            using PIN::GetPin;
            using PIN::ToggleEvery;
            

        private:
            

    };

    class BUTTON : PIN {
        public:
            BUTTON(uint pin, bool IsPullUp);

            bool IsPullUp();

            bool IsPressed();

            using PIN::GetState;
            using PIN::GetPin;
            using PIN::DisableIRQ;

            void SetIRQ(void (*func_ptr)(uint32_t), uint32_t eventMask) {
                this->func_ptr = func_ptr; 
                GPIO::PIN::SetIRQ<&GPIO::BUTTON::IRQCallback>(this, eventMask);
            }

        protected:
            void (*func_ptr)(uint32_t); //Store a pointer to a function

            void IRQCallback(uint32_t events) { //Use this in the IRQ to call a basic button function
                func_ptr(events);
            };
        private:

    };
} //Namespace GPIO

#endif