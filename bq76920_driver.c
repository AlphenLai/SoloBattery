#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

#include "ch.h"
#include "hal.h"

#include "bq76920_driver.h"

int GAIN;                     //GAIN = 365 μV/LSB + (ADCGAIN<4:0>in decimal) × (1 μV/LSB)
uint8_t ADCGAIN_value;        //gain value get from bq769 register
int8_t ADCOFFSET_value;       //ADC offset get from bq769 register
float batLeft;
float batMax;
double BatPercentage;
systime_t BatMon_thisEntry;   //beginning of mA average period
//systime_t BatMon_lastEntry;   //end of mA average period
double mA;                    //mA measurement from bq769
double avgmA;                 //average of measured mA within a period
double totalmA;               //sum of measured mA within a period
int mA_ctr;                   //number of measured mA within a period
double mA_offset;

//convert ADC gain from register value to approproate uV as mentioned in datasheet (uV)
int ADCGAINtoGain_uV(uint8_t ADCGAIN_hex) {
  if (ADCGAIN_hex > 0x1F)
    return 396;
  else
    return ADCGAIN_hex + 365;
}

//get voltage from coulomb counter
float CCtoVolt(int16_t ADC_cc) {
  return ADC_cc * (8.44 / 1000000.0);
}

// get voltage sum of all cells
float BATtoVolt(uint16_t ADC_bat, int numOfCell) {
  return ((4 * GAIN * ADC_bat) + (numOfCell * ADCOFFSET_value) * 1000.0) / 1000000.0;
}

//convert adc value to voltage 
float ADCtoVolt(uint16_t ADC_cell) {
  //V(cell) = (GAIN[μV/LSB] x ADC(cell) + OFFSET[mV] * 1000) / 1000000
  return (GAIN * ADC_cell + ADCOFFSET_value * 1000.0) / 1000000.0;
}
   
uint16_t VolttoADC(float volt) {
  return (volt * 1000000.0 - ADCOFFSET_value * 1000.0) / GAIN;
}

//get the current flow from the bq769
// @ RcurrSense    current sense resister (mOhm)
double GetCurFlow_mA(float RcurrSense) {
  //msg_t msg;
  int16_t cc_adc;
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, CC_HI, &cc_adc);
  // (((CC reading[V] / (RSNS[mohm] / 1000)[ohm]))[A] * 1000)[mA]
  return (CCtoVolt(cc_adc) / (RcurrSense/1000.0)) * 1000.0;
}

//get the battery percentage
double GetBatPercentage(void) {
  //the program enters this function when the bq769 alert is set, which means a new measurement has arrived and it must be handled.
  if(mA_ctr<=0) {
    //start of this average period
    BatMon_thisEntry = chVTGetSystemTime();
  }

  //get measured mA
  mA = GetCurFlow_mA(0.25);

  //sum mA
  totalmA += mA;
  mA_ctr++;
  
  if(mA_ctr>=10) {
    //calculate time interval of whole period
    sysinterval_t timeElapsed_t = chVTTimeElapsedSinceX(BatMon_thisEntry);
    uint32_t timeElapsed_ms = TIME_I2MS(timeElapsed_t);

    avgmA = totalmA / mA_ctr;

    batLeft -= avgmA * (timeElapsed_ms / 3600000.0);
    BatPercentage = (batLeft / batMax) * 100.0;
    
    //reset
    totalmA = 0;
    mA_ctr = 0;
    avgmA = 0;
  } 
  return BatPercentage;
}

//get voltage of each cells in the battery
void GetCellsVolt(float cellsVolt[]) {
  //msg_t msg;
  uint16_t cell_adc[5];
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC1_HI, &cell_adc[0]);
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC2_HI, &cell_adc[1]);
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC3_HI, &cell_adc[2]);
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC4_HI, &cell_adc[3]);
  I2CReadRegisterWordWithCRC(&I2CD1, addr_bq76920, VC5_HI, &cell_adc[4]);

  cellsVolt[0] = ADCtoVolt(cell_adc[0]);
  cellsVolt[1] = ADCtoVolt(cell_adc[1]);
  cellsVolt[2] = ADCtoVolt(cell_adc[2]);
  cellsVolt[3] = ADCtoVolt(cell_adc[3]);
  cellsVolt[4] = ADCtoVolt(cell_adc[4]);
}

//handle system fault when they appear
void SysFaultHandler(void) {
  //msg_t msg;
  uint8_t fault_mask = GetSysStat();
  if((fault_mask >> 5) && 0x01) {
    chThdSleepMilliseconds(3000);
    I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_STAT, 0b00100000);  
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

//calculate CRC for bq769
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

//initialize battery by writing all necessary register settings
void battery_init(int max_capacity) {
  uint8_t enableADCandTS1 = (1<<ADC_EN)|(1<<TEMP_SEL);
  uint8_t enableCellBalOnAllCells = 0b00011111;
  //uint8_t enableCCinOneShot = (0xFF&(0<<CC_EN))|(1<<CC_ONESHOT);
  uint8_t enableCCincontinuous = (1<<CC_EN);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL1, enableADCandTS1);  
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, enableCCincontinuous);
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, CELLBAL1, enableCellBalOnAllCells);
  batLeft = max_capacity;
  batMax = max_capacity;
}

//initialize bq769
void bq76920_init(void) {
  msg_t msg;
  uint8_t GAIN_buffer;
  
  //Set CC_CFG to 0x19 as mentioned in datasheet
  I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, CC_CFG, 0x19);

  //Set current and voltage protections for the bq769. Check datasheet for the table
  SetProtection(1, 4.2, 0x01, 3, 0x01, 0x02, 0x01, 0x07, 0x05);

  //get gain for ADC-to-voltage conversion
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCGAIN1, &GAIN_buffer);
  ADCGAIN_value = (GAIN_buffer &= 0b00001100) << 1;
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCGAIN2, &GAIN_buffer);
  ADCGAIN_value |= (GAIN_buffer &= 0b00011111);
  GAIN = ADCGAINtoGain_uV(ADCGAIN_value);

  //get offet for ADC-to-voltage conversion
  msg = I2CReadRegisterByteWithCRC(&I2CD1, addr_bq76920, ADCOFFSET, &ADCOFFSET_value);

  //get offset for current flow measurement
  ChangeBatteryStatus(Calibrate);
}

void ChangeBatteryStatus(char EN) {
  uint8_t newSysCtrl2;
  switch(EN) {
    case 0:
      //enable battery charge; disable battery discharge
      newSysCtrl2 = (1<<CC_EN)|(0<<DSG_ON)|(1<<CHG_ON);
      I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, newSysCtrl2);
      chThdSleepMilliseconds(1);
      break;
    case 1:
      //enable battery discharge; disable battery charge
      newSysCtrl2 = (1<<CC_EN)|(1<<DSG_ON)|(0<<CHG_ON);
      I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, newSysCtrl2);
      chThdSleepMilliseconds(1);
      break;
    case 2:
      //enable battery discharge; disable battery charge
      newSysCtrl2 = (1<<CC_EN)|(0<<DSG_ON)|(0<<CHG_ON);
      I2CWriteRegisterByteWithCRC(&I2CD1, addr_bq76920, SYS_CTRL2, newSysCtrl2);
      chThdSleepMilliseconds(1);
      mA_offset = GetCurFlow_mA(0.25);
      break;
  }
}

//reset alert pin
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