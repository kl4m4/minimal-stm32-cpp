#ifndef __LED_CONTROLLER_HPP
#define __LED_CONTROLLER_HPP

#include <stdint.h>

typedef enum {
    LED_INACTIVE,
    LED_ACTIVE
}eLedState;

class LedController {
    public:
        LedController(eLedState initial_state);
        eLedState led_state;
        void MakeActive();
        void MakeInactive();
    
    private:
        uint32_t magic_value;
};

#endif