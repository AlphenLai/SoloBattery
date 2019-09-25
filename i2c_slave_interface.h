#ifndef i2c_slave_interface_H_
#define i2c_slave_interface_H_

#define BATT_ADDR                     0x0B                ///< I2C address
#define BATT_ADDR_MIN                 0x08                ///< lowest possible address
#define BATT_ADDR_MAX                 0x7F                ///< highest possible address

#ifdef __cplusplus
extern "C" {
#endif
  void catchError(I2CDriver *i2cp);
  void notePoll(I2CDriver *i2cp);
  void messageProcessor(I2CDriver *i2cp);
  void clearAfterSend(I2CDriver *i2cp);
  void startComms(void);
#ifdef __cplusplus
}
#endif

#endif /* SMBUS_interface_H_ */