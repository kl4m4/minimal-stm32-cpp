extern "C" {
    #include "stm32f4xx_hal.h"
    #include "main.h"
}
#include "app.hpp"

void loop(){
    GPIO_PinState new_state;
    GPIO_PinState new_state_inverted;

    if(HAL_GPIO_ReadPin(B_USER_GPIO_Port, B_USER_Pin)){
      new_state = GPIO_PIN_SET;
      new_state_inverted = GPIO_PIN_RESET;
      //__ASM("nop");
    }else{
      new_state = GPIO_PIN_RESET;
      new_state_inverted = GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, new_state);
    HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, new_state_inverted);
}
