#include "led_controller.hpp"

extern "C" {
    #include "stm32f4xx_hal.h"
    #include "main.h"
}

constexpr uint32_t magic = 2345;

LedController::LedController(eLedState initial_state){
    led_state = initial_state;
    magic_value = magic;
}

void LedController::MakeActive(){
    led_state = LED_ACTIVE;
    HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_RESET);
}

void LedController::MakeInactive(){
    led_state = LED_INACTIVE;
    HAL_GPIO_WritePin(LED1_RED_GPIO_Port, LED1_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GREEN_GPIO_Port, LED2_GREEN_Pin, GPIO_PIN_SET);
}