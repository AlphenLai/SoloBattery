// #include <stdlib.h>
// #include <string.h>

// #include "ch.h"
// #include "hal.h"

// #include "i2c_slave_interface.h"
// #include "LED.h"

// I2CSlaveMsgCB messageProcessor, catchError, clearAfterSend, notePoll;

// const char hexString[16] = "0123456789abcdef";

// char initialReplyBody[8] = "76543210";

// uint32_t messageCounter = 0;                /* Counts number of messages received to return as part of response */

// uint8_t  rxBody[8];                       /* stores last message master sent us (intentionally a few bytes smaller than txBody) */
// uint8_t  txBody[8];                       /* Return message buffer for computed replies */

// // Message received handler
// const I2CSlaveMsg echoRx = {
//   sizeof(rxBody),       /* max sizeof received msg body */
//   rxBody,               /* body of received msg */
//   NULL,                 /* do nothing on address match */
//   messageProcessor,     /* Routine to process received messages */
//   catchError            /* Error hook */
// };

// // 'Empty' reply when nothing to say, and no message received. In RAM, to allow update
// I2CSlaveMsg initialReply = {
//   sizeof(initialReplyBody),  /* trailing zero byte will be repeated as needed */
//   (uint8_t *)initialReplyBody,
//   NULL,                 /* do nothing on address match */
//   notePoll,             /* Just register an event */
//   catchError            /* Error hook */
// };

// // Response to received messages
// I2CSlaveMsg echoReply = {  /* this is in RAM so size may be updated */
//   0,                    /* filled in with the length of the message to send */
//   txBody,               /* Response message */
//   NULL,                 /* do nothing special on address match */
//   clearAfterSend,       /* Clear receive buffer once replied */
//   catchError            /* Error hook */
// };

// /**
//  * Track I2C errors
//  */
// uint32_t lastI2cErrorFlags = 0;

// /**
//  * Generic error handler
//  *
//  * Called in interrupt context, so need to watch what we do
//  */
// void catchError(I2CDriver *i2cp) {
//   lastI2cErrorFlags = i2cp->errors;
//   chSysLockFromISR();
//   NULL,
//   chSysUnlockFromISR();
// }

// /**
//  * Note that we were polled (read request without preceding write)
//  *
//  * Called in interrupt context, so need to watch what we do
//  */
// void notePoll(I2CDriver *i2cp) {
//   (void) i2cp;
//   chSysLockFromISR();
//   NULL;
//   chSysUnlockFromISR();
// }

// void messageProcessor(I2CDriver *i2cp) {
//   Set_LED(0x08);
//   uint8_t i;
//   uint8_t *txPtr = txBody + 8;
//   size_t txLen;
//   uint32_t curCount;

//   size_t len = i2c_lld_get_slaveBytes(i2cp);         // Number of bytes received
//   if (len >= sizeof(rxBody))
//       len = sizeof(rxBody)-1;
//   rxBody[len]=0;                            // String termination sometimes useful

//   /* A real-world application would read and decode the message in rxBody, then generate an appropriate reply in txBody */
//   if (rxBody[0] == 0x0A)
//     txBody[0] = 0x0B;

//   curCount = ++messageCounter;
//   txLen = len + 11;                         // Add in the overhead

//   for (i = 0; i < 8; i++)
//   {
//     *--txPtr = hexString[curCount & 0xf];
//     curCount = curCount >> 4;
//   }

//   txPtr = txBody + 8;
//   *txPtr++ = ' ';
//   *txPtr++ = '[';
//   memcpy(txPtr, rxBody, len);               // Echo received message
//   txPtr += len;
//   *txPtr++ = ']';
//   *txPtr = '\0';

//   /** Message ready to go here */
//   echoReply.size = txLen;
//   chSysLockFromISR();
//   i2c_lld_slaveReply(i2cp, &echoReply);
//   NULL;
//   chSysUnlockFromISR();
// }

// /**
//  * Callback after sending of response complete - restores default reply in case polled
//  */
// void clearAfterSend(I2CDriver *i2cp)
// {
//   echoReply.size = 0;               // Clear receive message
//   i2c_lld_slaveReply(i2cp, &initialReply);
// }

// void startComms() {
//   I2CD1.slaveTimeout = TIME_INFINITE;       // Time for complete message

//   initialReply.size = strlen((char *)initialReply.body) + 1;                // Set the initial return message length
//   i2cSlaveConfigure(&I2CD1, &echoRx, &initialReply);

//   // Enable match address after everything else set up
//   i2cMatchAddress(&I2CD1, BATT_ADDR);
// }
