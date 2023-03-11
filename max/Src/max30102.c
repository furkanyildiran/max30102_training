#include "max30102.h"
#include "max30102_mnemonics.h"
#include "max30102_config.h"

static void write8(uint8_t reg_addr, uint8_t reg_val);
static uint8_t read8(uint8_t reg_addr);
static void readBurst(uint8_t reg_addr, uint8_t buff[], uint8_t buff_size);
static void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
static void Delay1Ms(void);


uint8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
uint8_t revisionID;
sense_struct sense;


uint8_t MAX30102_init(void) {
  // Step 1: Initial Communication and Verification
  // Check that a MAX30105 is connected
  if (MAX30102_readPartID() != MAX30102_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX30105 does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    return false;
  }
  MAX30102_setup(MAX30102_POWERLVL_CONF, MAX30102_SAMPAVG_CONF, MAX30102_LEDMODE_CONF, MAX30102_SAMPRATE_CONF, MAX30102_PULSEWIDTH_CONF, MAX30102_ADCRANGE_CONF);

  return true;
}


//Begin Interrupt configuration
uint8_t MAX30102_getINT1(void) {
	return read8(MAX30102_INTSTAT1);
}
uint8_t MAX30102_getINT2(void) {
	return read8(MAX30102_INTSTAT2);
}

void MAX30102_enableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102_disableAFULL(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102_enableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102_disableDATARDY(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102_enableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102_disableALCOVF(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102_enablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void MAX30102_disablePROXINT(void) {
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void MAX30102_enableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102_disableDIETEMPRDY(void) {
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30102_softReset(void) {
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    uint8_t response = read8(MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0) break; //We're done!
    Delay1Ms(); //Let's not over burden the I2C bus
  }
}

void MAX30102_shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102_wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102_setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102_setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102_setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102_setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102_setPulseAmplitudeRed(uint8_t amplitude) {
  write8(MAX30102_LED1_PULSEAMP, amplitude);
}

void MAX30102_setPulseAmplitudeIR(uint8_t amplitude) {
  write8(MAX30102_LED2_PULSEAMP, amplitude);
}

void MAX30102_setPulseAmplitudeGreen(uint8_t amplitude) {
  write8(MAX30102_LED3_PULSEAMP, amplitude);
}

void MAX30102_setPulseAmplitudeProximity(uint8_t amplitude) {
  write8(MAX30102_LED_PROX_AMP, amplitude);
}

void MAX30102_setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  write8(MAX30102_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30102_enableSlot(uint8_t slotNumber, uint8_t device) {
  switch (slotNumber) {
    case (1):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX30102_disableSlots(void) {
  write8(MAX30102_MULTILEDCONFIG1, 0);
  write8(MAX30102_MULTILEDCONFIG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30102_setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30102_clearFIFO(void) {
  write8(MAX30102_FIFOWRITEPTR, 0);
  write8(MAX30102_FIFOOVERFLOW, 0);
  write8(MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX30102_enableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30102_disableFIFORollover(void) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30102_setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX30102_getWritePointer(void) {
  return (read8(MAX30102_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX30102_getReadPointer(void) {
  return (read8(MAX30102_FIFOREADPTR));
}


// Die Temperature
// Returns temp in C
float MAX30102_readTemperature() {

  //DIE_TEMP_RDY interrupt must be enabled
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19

  // Step 1: Config die temperature register to take 1 temperature sample
  write8(MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    //uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!

	//Check to see if DIE_TEMP_RDY interrupt is set
	uint8_t response = read8(MAX30102_INTSTAT2);
    if ((response & MAX30102_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    Delay1Ms(); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = read8(MAX30102_DIETEMPINT);
  uint8_t tempFrac = read8(MAX30102_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30102_readTemperatureF() {
  float temp = MAX30102_readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// Set the PROX_INT_THRESHold
void MAX30102_setPROXINTTHRESH(uint8_t val) {
  write8(MAX30102_PROXINTTHRESH, val);
}


//
// Device ID and Revision
//
uint8_t MAX30102_readPartID() {
  return read8(MAX30102_PARTID);
}
void MAX30102_readRevisionID() {
  revisionID = read8(MAX30102_REVISIONID);
}
uint8_t MAX30102_getRevisionID() {
  return revisionID;
}
//Setup the sensor
//The MAX30105 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30105 sensor
void MAX30102_setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
	MAX30102_softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_2);
  else if (sampleAverage == 4) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_4);
  else if (sampleAverage == 8) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_8);
  else if (sampleAverage == 16) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_16);
  else if (sampleAverage == 32) MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_32);
  else MAX30102_setFIFOAverage(MAX30102_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  MAX30102_enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) MAX30102_setLEDMode(MAX30102_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) MAX30102_setLEDMode(MAX30102_MODE_REDIRONLY); //Red and IR
  else MAX30102_setLEDMode(MAX30102_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) MAX30102_setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) MAX30102_setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) MAX30102_setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) MAX30102_setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
  else MAX30102_setADCRange(MAX30102_ADCRANGE_2048);

  if (sampleRate < 100) MAX30102_setSampleRate(MAX30102_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_100);
  else if (sampleRate < 400) MAX30102_setSampleRate(MAX30102_SAMPLERATE_200);
  else if (sampleRate < 800) MAX30102_setSampleRate(MAX30102_SAMPLERATE_400);
  else if (sampleRate < 1000) MAX30102_setSampleRate(MAX30102_SAMPLERATE_800);
  else if (sampleRate < 1600) MAX30102_setSampleRate(MAX30102_SAMPLERATE_1000);
  else if (sampleRate < 3200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_1600);
  else if (sampleRate == 3200) MAX30102_setSampleRate(MAX30102_SAMPLERATE_3200);
  else MAX30102_setSampleRate(MAX30102_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_411); //18 bit resolution
  else MAX30102_setPulseWidth(MAX30102_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  MAX30102_setPulseAmplitudeRed(powerLevel);
  MAX30102_setPulseAmplitudeIR(powerLevel);
  MAX30102_setPulseAmplitudeGreen(powerLevel);
  MAX30102_setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  MAX30102_enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) MAX30102_enableSlot(2, SLOT_IR_LED);
  if (ledMode > 2) MAX30102_enableSlot(3, SLOT_GREEN_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  MAX30102_clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX30102_available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value
uint32_t MAX30102_getRed(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30102_safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t MAX30102_getIR(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30102_safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX30102_getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30102_safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report te next Red value in the FIFO
uint32_t MAX30102_getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO
uint32_t MAX30102_getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX30102_getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

//Advance the tail
void MAX30102_nextSample(void)
{
  if(MAX30102_available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX30102_check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = MAX30102_getReadPointer();
  uint8_t writePointer = MAX30102_getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    if (bytesLeftToRead > I2C_BUFFER_LENGTH)
    {
    	//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        bytesLeftToRead = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
    }
    uint8_t buff[180]={0};
    readBurst(MAX30102_FIFODATA, buff, bytesLeftToRead);
    uint8_t counter;
    while(bytesLeftToRead){
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = buff[counter++];
        temp[1] = buff[counter++];
        temp[0] = buff[counter++];

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));

		tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = buff[counter++];
          temp[1] = buff[counter++];
          temp[0] = buff[counter++];

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

		  tempLong &= 0x3FFFF; //Zero out all but 18 bits

		  sense.IR[sense.head] = tempLong;
        }


        bytesLeftToRead -= (activeLEDs * 3);

    }


  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
uint8_t MAX30102_safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = HAL_GetTick();
  while(1)
  {
	if(HAL_GetTick() - markTime > maxTimeToCheck) return(false);

	if(MAX30102_check() == true) //We found new data!
	  return(true);
	Delay1Ms();
  }
}


static void bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  uint8_t originalContents = read8(reg);
  originalContents = originalContents & mask;
  write8(reg, originalContents | thing);
}
static void Delay1Ms(void){
	HAL_Delay(1);
}
static void write8(uint8_t reg_addr, uint8_t reg_val){
	I2C1_Write8((MAX30105_ADDRESS << 1), reg_addr, reg_val);
}
static uint8_t read8(uint8_t reg_addr){
	return I2C1_Read8(((MAX30105_ADDRESS << 1) | 1), reg_addr);
}
static void readBurst(uint8_t reg_addr, uint8_t buff[], uint8_t buff_size){
	I2C1_ReadBurst(((MAX30105_ADDRESS << 1) | 1), reg_addr, buff, buff_size);
}
