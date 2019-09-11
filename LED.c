#include <stdlib.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"

#include "LED.h"

uint8_t debug_LED = 0;

void LED_init(void) {
    palSetPad(GPIOA, GPIOA_LED1);
    palSetPad(GPIOA, GPIOA_LED2);
    palSetPad(GPIOA, GPIOA_LED3);
    palSetPad(GPIOB, GPIOB_LED4);
    palSetPad(GPIOB, GPIOB_LED5);
}

void SetDebugLED(uint8_t bit) {
    debug_LED = bit;
}

void Set_LED(uint8_t LED_mask) {
    if(LED_mask & 0b00000001 & ~debug_LED) {
        palClearPad(GPIOA, GPIOA_LED1);
    } else {
        palSetPad(GPIOA, GPIOA_LED1);
    }
    if(LED_mask & 0b00000010 & ~debug_LED) {
        palClearPad(GPIOA, GPIOA_LED2);
    } else {
        palSetPad(GPIOA, GPIOA_LED2);
    }
    if(LED_mask & 0b00000100 & ~debug_LED) {
        palClearPad(GPIOA, GPIOA_LED3);
    } else {
        palSetPad(GPIOA, GPIOA_LED3);
    }
    if(LED_mask & 0b00001000 & ~debug_LED) {
        palClearPad(GPIOB, GPIOB_LED4);
    } else {
        palSetPad(GPIOB, GPIOB_LED4);
    }
    if(LED_mask & 0b00010000 & ~debug_LED) {
        palClearPad(GPIOB, GPIOB_LED5);
    } else {
        palSetPad(GPIOB, GPIOB_LED5);
    }
}

void Show_BatPercentage(float battery_percentage) {
    if (battery_percentage >= 99) {
        Set_LED(0b00001111);
    } else if (battery_percentage >= 95) {
        Set_LED(0b00001110);
    } else if (battery_percentage >= 90) {
        Set_LED(0b00001100);
    } else if (battery_percentage >= 80) {
        Set_LED(0b00001000);
    } else if (battery_percentage >= 70) {
        Set_LED(0b00000000);
    }
}