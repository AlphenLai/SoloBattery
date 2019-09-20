#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "ch.h"
#include "hal.h"

#include "bq76920_driver.h"

#define ST2MS(n) (((((n) - 1UL) * 1000UL) / CH_CFG_ST_FREQUENCY) + 1UL)

int GAIN;               /* GAIN = 365 μV/LSB + (ADCGAIN<4:0>in decimal) × (1 μV/LSB) */
uint8_t ADCGAIN_value;
int8_t ADCOFFSET_value;
volatile float BatLeft;
float BatMax;
systime_t BatMon_lastEntry;

int TwoSComplement(uint16_t raw) {
  if(raw>>7)
    return -(~raw + 1);
  else
    return raw;
}

int ADCGAINtoDec(uint8_t ADCGAIN_hex) {
  if (ADCGAIN_hex > 0x1F)
    return 396;
  else
    return ADCGAIN_hex + 365;
}

float CCtoVolt(int16_t ADC_cc) {
  return ADC_cc * (8.44 / 1000000.0);
}

// Sum of all cells
float BATtoVolt(uint16_t ADC_bat, int numOfCell) {
  return ((4 * GAIN * ADC_bat) + (numOfCell * ADCOFFSET_value) * 1000.0) / 1000000.0;
}

float ADCtoVolt(uint16_t ADC_cell) {
  //V(cell) = (GAIN[μV/LSB] x ADC(cell) + OFFSET[mV] * 1000) / 1000000
  return (GAIN * ADC_cell + ADCOFFSET_value * 1000.0) / 1000000.0;
}

uint16_t VolttoADC(float volt) {
  return (volt * 1000000.0 - ADCOFFSET_value * 1000.0) / GAIN;
}

// @ RcurrSense    current sense resister (mOhm)
double GetCurFlow_mA(float RcurrSense) {
  msg_t msg;
  int16_t cc_adc;
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, CC_HI, &cc_adc);
  // (((CC reading[V] / (RSNS[mohm] / 1000)[ohm]))[A] * 1000)[mA]
  return (CCtoVolt(cc_adc) / (RcurrSense/1000.0)) * 1000.0;
}

double GetBatPercentage() {
  //volatile systime_t timeElapsed_t = BatMon_thisEntry - BatMon_lastEntry;
  systime_t BatMon_thisEntry = chVTGetSystemTime();
  volatile sysinterval_t timeElapsed_t = chVTTimeElapsedSinceX(BatMon_lastEntry);
  //volatile uint32_t timeElapsed_ms = ST2MS(timeElapsed_t);
  uint32_t timeElapsed_ms = TIME_I2MS(timeElapsed_t);
  volatile double mA = GetCurFlow_mA(0.25);
  BatLeft -= mA * (timeElapsed_ms / 3600000.0);
  volatile double BatPercentage = (BatLeft / BatMax) * 100.0;
  
  BatMon_lastEntry = BatMon_thisEntry;
  return BatPercentage;
}

void GetCellsVolt(float cellsVolt[]) {
  msg_t msg;
  uint16_t cell_adc[5];
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC1_HI, &cell_adc[0]);
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC2_HI, &cell_adc[1]);
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC3_HI, &cell_adc[2]);
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC4_HI, &cell_adc[3]);
  msg = I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC5_HI, &cell_adc[4]);

  cellsVolt[0] = ADCtoVolt(cell_adc[0]);
  cellsVolt[1] = ADCtoVolt(cell_adc[1]);
  cellsVolt[2] = ADCtoVolt(cell_adc[2]);
  cellsVolt[3] = ADCtoVolt(cell_adc[3]);
  cellsVolt[4] = ADCtoVolt(cell_adc[4]);
}

void SysFaultHandler(void) {
  msg_t msg;
  uint8_t fault_mask = GetSysStat();
  if((fault_mask >> 5) && 0x01) {
    chThdSleepMilliseconds(3000);
    msg = I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_STAT, 0b00100000);  
  }  
}

/*
bit 7 CC_READY        fresh coulomb counter reading is available 
bit 6 RSVD            Reserved. Do not use
bit 5 DEVICE_XREADY   Internal chip fault indicator
bit 4 OVRD_ALERT      External pull-up on the ALERT pin indicator
bit 3 UV              Undervoltage fault event indicator
bit 2 OV              Overvoltage fault event indicator
bit 1 SCD             Short circuit in discharge fault event indicator
bit 0 OCD             Over current in discharge fault event indicator
*/
uint8_t GetSysStat(void) {
  uint8_t SysStatReg = 0;
  I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_STAT, SysStatReg);
  return SysStatReg;
}

uint8_t CRC8(uint8_t *ptr, uint8_t len, uint8_t key) {
	uint8_t i;
	uint8_t crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
}

msg_t I2CWriteRegisterByteWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint8_t data) {
  msg_t msg;
  i2cflags_t err;
  uint8_t tx_buffer[3];
  uint8_t CRCInput[3];

  //generate CRC
  CRCInput[0]=dev_address << 1;
  CRCInput[1]=reg_address;
  CRCInput[2]=data;

  //generate data sequence
  tx_buffer[0] = reg_address;
  tx_buffer[1] = data;
  tx_buffer[2] = CRC8(CRCInput, 3, 7);

  //write data to i2c bus
  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, addr_bq76920, tx_buffer, sizeof(tx_buffer),
                                  NULL, 0, TIME_MS2I(4));
  i2cReleaseBus(i2cp);
  if (msg != MSG_OK)
    err = i2cGetErrors(i2cp);
  return msg;
}

msg_t I2CReadRegisterByteWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint8_t *data) {
  msg_t msg;
  i2cflags_t err;
  uint8_t tx_buffer[1] = {reg_address};
  uint8_t rx_buffer[2];
  uint8_t CRCInput[2];
  uint8_t crc;

  //read data from i2c bus
  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(i2cp, addr_bq76920, tx_buffer, 1,
                                rx_buffer, 2, TIME_MS2I(4));
  i2cReleaseBus(i2cp);
  if (msg != MSG_OK)
    err = i2cGetErrors(i2cp);

  //CRC check
  CRCInput[0] = (addr_bq76920 << 1) + 1;
  CRCInput[1] = rx_buffer[0];
  crc = CRC8(CRCInput, 2, 7);
  if (crc != rx_buffer[1])
    palSetPad(GPIOA, GPIOA_LED2);
  else
    *data = rx_buffer[0];
  return msg;
}

msg_t I2CReadRegisterWordWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint16_t *data) {
  msg_t msg;
  i2cflags_t err;
  uint8_t tx_buffer[1] = {reg_address};
  uint8_t rx_buffer[4];
  uint8_t CRCInput[2];
  uint8_t crc[2];

  //read data from i2c bus
  i2cAcquireBus(i2cp);
  msg = i2cMasterTransmitTimeout(&I2CD1, addr_bq76920, tx_buffer, 1,
                                  rx_buffer, 4, TIME_MS2I(4));
  i2cReleaseBus(i2cp);
  if (msg != MSG_OK)
    err = i2cGetErrors(i2cp);

  //CRC check for first byte
  CRCInput[0] = (addr_bq76920 << 1) + 1;
  CRCInput[1] = rx_buffer[0];
  crc[0] = CRC8(CRCInput, 2, 7) ;
  //CRC check for second byte
  CRCInput[0] = rx_buffer[2];
  crc[1] = CRC8(CRCInput, 1, 7);
  if (crc[0] != rx_buffer[1] || crc[1]!= rx_buffer[3] )
    return -1;
  else 
    *data = (rx_buffer[0]<<8) + rx_buffer[2];
  return msg;
}

void battery_init(int max_capacity) {
  uint8_t enableADCandTS1 = 0|(1<<ADC_EN)|(1<<TEMP_SEL);
  uint8_t enableCellBalOnAllCells = 0b00011111;
  uint8_t enableCCinOneShot = (0xFF&(0<<CC_EN))|(1<<CC_ONESHOT);
  uint8_t enableCCincontinuous = 0|(1<<CC_EN);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL1, enableADCandTS1);  
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, enableCCincontinuous);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, CELLBAL1, enableCellBalOnAllCells);
  BatLeft = max_capacity;
  BatMax = max_capacity;
}

void bq76920_init(void) {
  msg_t msg;
  uint8_t GAIN_buffer;
  
  //Set CC_CFG to 0x19 as mentioned in datasheet
  msg = I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, CC_CFG, 0x19);

  SetProtection(1, 4.2, 0x01, 3, 0x01, 0x02, 0x01, 0x07, 0x05);

  //get gain for ADC-to-voltage conversion
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCGAIN1, &GAIN_buffer);
  ADCGAIN_value = (GAIN_buffer &= 0b00001100) << 1;
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCGAIN2, &GAIN_buffer);
  ADCGAIN_value |= (GAIN_buffer &= 0b00011111);
  GAIN = ADCGAINtoDec(ADCGAIN_value);

  //get offet for ADC-to-voltage conversion
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCOFFSET, &ADCOFFSET_value);
}

void ChargeEN(void) {
  //(1<<CC_EN)|(0<<DSG_ON)|(1<<CHG_ON)
  uint8_t newSysCtrl2 = 0b01000001;
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, newSysCtrl2);
  chThdSleepMilliseconds(1);
}
void DischargeEN(void) {
  //(1<<CC_EN)|(1<<DSG_ON)|(0<<CHG_ON)
  uint8_t newSysCtrl2 = 0b01000010;
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, newSysCtrl2);
  chThdSleepMilliseconds(1);
}

void ResetAlert(void) {
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_STAT, 0b00010000);
}

//Please be noticed that delay values are lookup from datasheet
//unit of delay and current parameters are *not time*
void SetProtection(uint8_t RSNS, float OV, uint8_t OV_delay, float UV, uint8_t UV_delay, uint8_t SCD, uint8_t SCD_delay, uint8_t OCD, uint8_t OCD_delay) {
  //PROTECT1
  //Set RSNS, SCD_D, SCD_T
  uint8_t PROTECT1_value = (RSNS << 7)|(SCD_delay << 3)|(SCD << 0);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, PROTECT1, PROTECT1_value);

  //PROTECT2
  //Set OCD_D, OCD_T
  uint8_t PROTECT2_value = (OCD_delay << 4)|(OCD << 0);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, PROTECT1, PROTECT2_value);

  //PROTECT3
  //Set UV_D, OV_D
  uint8_t PROTECT3_value = (UV_delay << 6)|(OV_delay << 4);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, PROTECT3, PROTECT3_value);

  //Set OV_TRIP and UV_TRIP
  uint8_t OV_TRIP_value = VolttoADC(OV) >> 4;
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, OV_TRIP, OV_TRIP_value);
  uint8_t UV_TRIP_value = VolttoADC(UV) >> 4;
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, UV_TRIP, UV_TRIP_value);
}