#ifndef __MAX30102_H__
#define __MAX30102_H__
#define MAX30105_ADDRESS          	0x57 //7-bit I2C Address
#define I2C_BUFFER_LENGTH 			32
#include <stdint.h>
#include <stdlib.h>
#include "stm32wbxx_hal.h"
#include <string.h>
#include "i2c1.h"

uint8_t MAX30102_init(void);
uint32_t MAX30102_getRed(void); //Returns immediate red value
uint32_t MAX30102_getIR(void); //Returns immediate IR value
uint32_t MAX30102_getGreen(void); //Returns immediate green value
uint8_t MAX30102_safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

// Configuration
void MAX30102_softReset();
void MAX30102_shutDown();
void MAX30102_wakeUp();
void MAX30102_setLEDMode(uint8_t mode);
void MAX30102_setADCRange(uint8_t adcRange);
void MAX30102_setSampleRate(uint8_t sampleRate);
void MAX30102_setPulseWidth(uint8_t pulseWidth);
void MAX30102_setPulseAmplitudeRed(uint8_t value);
void MAX30102_setPulseAmplitudeIR(uint8_t value);
void MAX30102_setPulseAmplitudeGreen(uint8_t value);
void MAX30102_setPulseAmplitudeProximity(uint8_t value);
void MAX30102_setProximityThreshold(uint8_t threshMSB);
//Multi-led configuration mode (page 22)
void MAX30102_enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
void MAX30102_disableSlots(void);

// Data Collection

//Interrupts (page 13, 14)
uint8_t MAX30102_getINT1(void); //Returns the main interrupt group
uint8_t MAX30102_getINT2(void); //Returns the temp ready interrupt
void MAX30102_enableAFULL(void); //Enable/disable individual interrupts
void MAX30102_disableAFULL(void);
void MAX30102_enableDATARDY(void);
void MAX30102_disableDATARDY(void);
void MAX30102_enableALCOVF(void);
void MAX30102_disableALCOVF(void);
void MAX30102_enablePROXINT(void);
void MAX30102_disablePROXINT(void);
void MAX30102_enableDIETEMPRDY(void);
void MAX30102_disableDIETEMPRDY(void);

//FIFO Configuration (page 18)
void MAX30102_setFIFOAverage(uint8_t samples);
void MAX30102_enableFIFORollover();
void MAX30102_disableFIFORollover();
void MAX30102_setFIFOAlmostFull(uint8_t samples);
//FIFO Reading
uint16_t MAX30102_check(void); //Checks for new data and fills FIFO
uint8_t MAX30102_available(void); //Tells caller how many new samples are available (head - tail)
void MAX30102_nextSample(void); //Advances the tail of the sense array
uint32_t MAX30102_getFIFORed(void); //Returns the FIFO sample pointed to by tail
uint32_t MAX30102_getFIFOIR(void); //Returns the FIFO sample pointed to by tail
uint32_t MAX30102_getFIFOGreen(void); //Returns the FIFO sample pointed to by tail


void MAX30102_clearFIFO(void); //Sets the read/write pointers to zero

//Proximity Mode Interrupt Threshold
void MAX30102_setPROXINTTHRESH(uint8_t val);

// Die Temperature
float MAX30102_readTemperature();
float MAX30102_readTemperatureF();
// Detecting ID/Revision
uint8_t MAX30102_getRevisionID();
uint8_t MAX30102_readPartID();

// Setup the IC with user selectable settings
void MAX30102_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.

void MAX30102_readRevisionID();
uint8_t MAX30102_getWritePointer(void);
uint8_t MAX30102_getReadPointer(void);


#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
typedef struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
} sense_struct; //This is our circular buffer of readings from the sensor



#endif
