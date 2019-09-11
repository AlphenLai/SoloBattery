/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "bq76920_driver.h"
#include "LED.h"

#include <math.h>

#define battery_mAh         4750

#define ADC1_NUM_CHANNELS   1
#define ADC1_BUF_DEPTH      8

uint16_t TS1;
uint16_t RT2_FLT;
float cellsVolt[5];
volatile float battery_percentage;

static adcsample_t RT2_adc[ADC1_NUM_CHANNELS * ADC1_BUF_DEPTH];  //12bit

/*
 * ADC streaming callback.
 */
 
static void adccallback(ADCDriver *adcp) {

  if (adcIsBufferComplete(adcp)) {
  }    
}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * input clock = 24MHz.
 */
static const I2CConfig i2cfg1 = {
  STM32_TIMINGR_PRESC(5U) |   //24MHz/3 = 8MHz I2CCLK.
  STM32_TIMINGR_SCLDEL(0U) | STM32_TIMINGR_SDADEL(0U) |
  STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(18U),    //STM32 I2C time registers
  0, //DNF = 15
  0
};

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN1.
 */
static const ADCConversionGroup adccfg1 = {
 FALSE,
 ADC1_NUM_CHANNELS,
 NULL,
 NULL,
 ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
 ADC_TR(0, 0),                                     /* TR */
 ADC_SMPR_SMP_1P5,                                 /* SMPR */
 ADC_CHSELR_CHSEL1                                /* CHSELR */
};

/*
 * LEDs batLED thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("batLED");
  while (true) {
    chThdSleepMilliseconds(500);
    palSetPad(GPIOB, GPIOB_LED5);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOB, GPIOB_LED5);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Starting the I2C driver 1.
   */
  i2cStart(&I2CD1, &i2cfg1);
  
  /*
   * Creates the batLED thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Activates the ADC1 driver and the temperature sensor.
   */
   adcStart(&ADCD1, NULL);
   adcSTM32SetCCR(ADC_CCR_TSEN | ADC_CCR_VREFEN);

  chThdSleepMilliseconds(1000);

  //turn on all LEDs
  palClearPad(GPIOA, GPIOA_LED1);
  palClearPad(GPIOA, GPIOA_LED2);
  palClearPad(GPIOA, GPIOA_LED3);
  palClearPad(GPIOB, GPIOB_LED4);
  palClearPad(GPIOB, GPIOB_LED5);

  chThdSleepMilliseconds(1000);
  //turn off all LEDs
  palSetPad(GPIOA, GPIOA_LED1);
  palSetPad(GPIOA, GPIOA_LED2);
  palSetPad(GPIOA, GPIOA_LED3);
  palSetPad(GPIOB, GPIOB_LED4);
  palSetPad(GPIOB, GPIOB_LED5);

  //initialize bq76920
  bq76920_init();
  battery_init(battery_mAh);
  chThdSleepMilliseconds(100);

  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (true) {
    msg_t msg;
    //i2cflags_t err;
    DischargeEN();
    
    //the hotter temperature, the smaller the value
    msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, TS1_HI, &TS1);
    adcStartConversion(&ADCD1, &adccfg1, RT2_adc, ADC1_BUF_DEPTH);
    RT2_FLT = RT2_adc[0];

    GetCellsVolt(cellsVolt);

    if (palReadPad(GPIOA, GPIOA_ALERT))
    {
      ResetAlert();
      volatile systime_t now = chVTGetSystemTime();
      battery_percentage = GetBatPercentage(now);
    }

    chThdSleepMilliseconds(500);
    if (battery_percentage >= 99) {
      //100-99.1
      palClearPad(GPIOA, GPIOA_LED1);
      palClearPad(GPIOA, GPIOA_LED2);
      palClearPad(GPIOA, GPIOA_LED3);
      palClearPad(GPIOB, GPIOB_LED4);
    } else if (battery_percentage >= 95) {
      //99-95
      palSetPad(GPIOA, GPIOA_LED1);
      palClearPad(GPIOA, GPIOA_LED2);
      palClearPad(GPIOA, GPIOA_LED3);
      palClearPad(GPIOB, GPIOB_LED4);
    } else if (battery_percentage >= 90) {
      //94-90
      palSetPad(GPIOA, GPIOA_LED1);
      palSetPad(GPIOA, GPIOA_LED2);
      palClearPad(GPIOA, GPIOA_LED3);
      palClearPad(GPIOB, GPIOB_LED4);
    } else if (battery_percentage >= 80) {
      //89-80
      palSetPad(GPIOA, GPIOA_LED1);
      palSetPad(GPIOA, GPIOA_LED2);
      palSetPad(GPIOA, GPIOA_LED3);
      palClearPad(GPIOB, GPIOB_LED4);
    } else if (battery_percentage >= 70) {
      //79-70
      palSetPad(GPIOA, GPIOA_LED1);
      palSetPad(GPIOA, GPIOA_LED2);
      palSetPad(GPIOA, GPIOA_LED3);
      palSetPad(GPIOB, GPIOB_LED4);
      ChargeEN();
    }
    chThdSleepMilliseconds(500);
    
    // if (palReadPad(GPIOA, GPIOA_WKUP1)) {
    //   palClearPad(GPIOA, GPIOA_LED1);
    //   palClearPad(GPIOA, GPIOA_LED2);
    //   palClearPad(GPIOA, GPIOA_LED3);
    //   palClearPad(GPIOB, GPIOB_LED4);
    //   palClearPad(GPIOB, GPIOB_LED5);
    //   chThdSleepMilliseconds(500);

    //   if(battery_percentage < 98) {
    //     palSetPad(GPIOA, GPIOA_LED1);
    //   }
    //   chThdSleepMilliseconds(500);
    //   if(battery_percentage < 96) {
    //     palSetPad(GPIOA, GPIOA_LED2);
    //   }
    //   chThdSleepMilliseconds(500);
    //   if(battery_percentage < 94) {
    //     palSetPad(GPIOA, GPIOA_LED3);
    //   }
    //   chThdSleepMilliseconds(500);
    //   if(battery_percentage < 40) {
    //     palSetPad(GPIOB, GPIOB_LED4);
    //   }
    //   chThdSleepMilliseconds(500);
    //   if(battery_percentage < 20) {
    //     palSetPad(GPIOB, GPIOB_LED5);
    //   }
    //   chThdSleepMilliseconds(500);
      
      palSetPad(GPIOA, GPIOA_LED1);
      palSetPad(GPIOA, GPIOA_LED2);
      palSetPad(GPIOA, GPIOA_LED3);
      palSetPad(GPIOB, GPIOB_LED4);
      palSetPad(GPIOB, GPIOB_LED5);
    //}
  }
}
