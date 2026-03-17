#ifndef PWM_H
#define PWM_H //AI help me notice the spelling error here. It broke alot but very unclearly, you should try it.

#include "GPIO.h"
#include <cassert>

namespace PWM
{

    class PIN : public GPIO::PIN{

        public: 
            PIN(uint pin, int frequency, int wrapCounter);

            virtual void FadeUp(int msecs, float peak, volatile std::atomic<int>& DiffVar);
            virtual void FadeDown(int msecs, float peak, volatile std::atomic<int>& DiffVar);
            
            virtual void SetDuty(uint duty);
            virtual void SetDuty(float duty);

            virtual void Stop();

            virtual void Toggle() override;
            virtual void SetState(bool IsOn) override;
            virtual bool GetState() override;
            virtual float GetDuty();
            
            using GPIO::PIN::GetPin;
            using GPIO::PIN::ToggleEvery;
            using GPIO::PIN::SetIRQ;
            using GPIO::PIN::DisableIRQ;

            virtual ~PIN() {Stop();};

        protected:
            PIN() = delete; //Remove Default Constructor
            float currentDuty;
            int FREQUENCY;
            uint CHANNEL;
            uint SLICE;
            int WRAPCOUNTER;

    };

    class LED : PIN{

        public:
            LED(uint pin);

            using PIN::Toggle;
            using PIN::SetState;
            using PIN::FadeDown;
            using PIN::FadeUp;
            using PIN::SetDuty;
            using PIN::Stop;
            using PIN::GetPin;
            using PIN::ToggleEvery;

        private:


    };

    class MOTOR : PIN {

        public:
            MOTOR(uint pwmPin, uint pin1, uint pin2);

            using PIN::GetDuty;
            using PIN::Stop;

            virtual void SetSleep(bool state);

            virtual void Forward(float speed);
            virtual void Backward(float speed);

            virtual ~MOTOR() {};
        protected:
            
            using PIN::SetDuty;
            using PIN::GetPin;

            GPIO::PIN PHPin;
            GPIO::PIN SleepPin;


    };
} // namespace PWM

#endif