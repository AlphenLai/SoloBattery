#ifndef BQ76920_DRIVER_H_
#define BQ76920_DRIVER_H_

/* device I2C address */
#define addr_bq76920 0x08

/* slave specific addresses */
#define SYS_STAT     0x00

/* 
SYS_STAT bit
@ 0 = Fresh CC reading not yet available or bit is cleared by host microcontroller.
@ 1 = Fresh CC reading is available. Remains latched high until cleared by host.
Bits in SYS_STAT may be cleared by writing a "1" to the corresponding bit. 
*/
#define CC_READY_flag   7
#define RSVD_flag       6
#define DEVICE_XREADY_flag 5
#define OVRD_ALERT_flag 4
#define UV_flag         3
#define OV_flag         2
#define SCD_flag        1
#define OCD_flag        0

#define CELLBAL1     0x01     /* set to 0b00011111 to enable cell balancing on cell 1-5 */
#define CELLBAL2     0x02
#define CELLBAL3     0x03
#define SYS_CTRL1    0x04

/* 
SYS_CTRL1 bit
@ 0 = Disable voltage and temperature ADC readings (also disables OV protection)
@ 1 = Enable voltage and temperature ADC readings (also enables OV protection)
*/
#define ADC_EN       4

/* 
SYS_CTRL1 bit
@ 0 = Store internal die temperature voltage reading in TSx_HI and TSx_LO
@ 1 = Store thermistor reading in TSx_HI and TSx_LO (all thermistor ports) 
*/
#define TEMP_SEL     3

#define SYS_CTRL2    0x05

/* 
SYS_CTRL2 bit
@ 0 = Disable CC continuous readings
@ 1 = Enable CC continuous readings and ignore [CC_ONESHOT] state 
*/
#define CC_EN        6

/* 
SYS_CTRL2 bit
0 = No action
1 = Enable single CC reading (only valid if [CC_EN] = 0), and [CC_READY] = 0) 
*/
#define CC_ONESHOT   5

#define DSG_ON       1

#define CHG_ON       0

#define PROTECT1     0x06
#define PROTECT2     0x07
#define PROTECT3     0x08
#define OV_TRIP      0x09
#define UV_TRIP      0x0A
#define CC_CFG       0x0B
#define VC1_HI       0x0C
#define VC1_LO       0x0D
#define VC2_HI       0x0E
#define VC2_LO       0x0F
#define VC3_HI       0x10
#define VC3_LO       0x11
#define VC4_HI       0x12
#define VC4_LO       0x13
#define VC5_HI       0x14
#define VC5_LO       0x15
#define BAT_HI       0x2A
#define BAT_LO       0x2B
#define TS1_HI       0x2C
#define TS1_LO       0x2D
#define CC_HI        0x32
#define CC_LO        0x33
#define ADCGAIN1     0x50
#define ADCOFFSET    0x51
#define ADCGAIN2     0x59

#define VC1_HI       0x0C
#define VC1_LO       0x0D
#define VC2_HI       0x0E
#define VC2_LO       0x0F
#define VC3_HI       0x10
#define VC3_LO       0x11
#define VC4_HI       0x12
#define VC4_LO       0x13
#define VC5_HI       0x14
#define VC5_LO       0x15
#define BAT_HI       0x2A
#define BAT_LO       0x2B
#define TS1_HI       0x2C
#define TS1_LO       0x2D
#define ADCGAIN1     0x50
#define ADCGAIN2     0x59
#define ADCOFFSET    0x51

#define Calibrate      0
#define Charge         1
#define Discharge      2

#ifdef __cplusplus
extern "C" {
#endif
  int ADCGAINtoGain_uV(uint8_t ADCGAIN_hex);
  float CCtoVolt(int16_t ADC_cc);
  float BATtoVolt(uint16_t ADC_bat, int numOfCell);
  float ADCtoVolt(uint16_t ADC_cell);
  uint16_t VolttoADC(float volt);
  double GetCurFlow_mA(float RcurrSense);
  double GetBatPercentage(void);
  void GetCellsVolt(float cellsVolt[]);
  void SysFaultHandler(void);
  uint8_t GetSysStat(void);
  uint8_t CRC8(unsigned char *ptr, unsigned char len, unsigned char key);
  msg_t I2CWriteRegisterByteWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint8_t data);
  msg_t I2CReadRegisterByteWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint8_t *data);
  msg_t I2CReadRegisterWordWithCRC(I2CDriver *i2cp, uint8_t dev_address, uint8_t reg_address, uint16_t *data);
  void battery_init(int max_capacity);
  void bq76920_init(void);
  void ChangeBatteryStatus(char EN);
  bool CheckLoadPresent(void);
  void ResetFlag(uint8_t bit);
  void SetProtection(uint8_t RSNS, float OV, uint8_t OV_delay, float UV, uint8_t UV_delay, uint8_t SCD, uint8_t SCD_delay, uint8_t OCD, uint8_t OCD_delay);
#ifdef __cplusplus
}
#endif

#endif /* BQ76920_DRIVER_H_ */
