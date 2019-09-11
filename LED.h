#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif
  void LED_init(void);
  void SetDebugLED(uint8_t bit);
  void Set_LED(uint8_t LED_mask);
  void Show_BatPercentage(float percentage);
#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */