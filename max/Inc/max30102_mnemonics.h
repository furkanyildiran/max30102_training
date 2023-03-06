#ifndef __MAX_MNEMONICS_H_
#define __MAX_MNEMONICS_H_

// Status Registers
#define MAX30105_INTSTAT1 									0x00
#define MAX30105_INTSTAT2 									0x01
#define MAX30105_INTENABLE1					 				0x02
#define MAX30105_INTENABLE2			 						0x03

// FIFO Registers
#define MAX30105_FIFOWRITEPTR							  	0x04
#define MAX30105_FIFOOVERFLOW  								0x05
#define MAX30105_FIFOREADPTR		  						0x06
#define MAX30105_FIFODATA 									0x07
// Configuration Registers
#define MAX30105_FIFOCONFIG			  						0x08
#define MAX30105_MODECONFIG 			 					0x09
#define MAX30105_PARTICLECONFIG 					 		0x0A    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
#define MAX30105_LED1_PULSEAMP  							0x0C
#define MAX30105_LED2_PULSEAMP 								0x0D
#define MAX30105_LED3_PULSEAMP  							0x0E
#define MAX30105_LED_PROX_AMP  								0x10
#define MAX30105_MULTILEDCONFIG1 							0x11
#define MAX30105_MULTILEDCONFIG2 							0x12

// Die Temperature Registers
#define MAX30105_DIETEMPINT			  						0x1F
#define MAX30105_DIETEMPFRAC  								0x20
#define MAX30105_DIETEMPCONFIG  							0x21

// Proximity Function Registers
#define MAX30105_PROXINTTHRESH 	 							0x30

// Part ID Registers
#define MAX30105_REVISIONID  								0xFE
#define MAX30105_PARTID  									0xFF    // Should always be 0x15. Identical to MAX30102.
// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
#define MAX30105_INT_A_FULL_MASK 							(uint8_t)(~0b10000000)
#define MAX30105_INT_A_FULL_ENABLE  						0x80
#define MAX30105_INT_A_FULL_DISABLE  						0x00

#define MAX30105_INT_DATA_RDY_MASK  						(uint8_t)(~0b01000000)
#define MAX30105_INT_DATA_RDY_ENABLE 						0x40
#define MAX30105_INT_DATA_RDY_DISABLE		 				0x00

#define MAX30105_INT_ALC_OVF_MASK  							(uint8_t)(~0b00100000)
#define MAX30105_INT_ALC_OVF_ENABLE  						0x20
#define MAX30105_INT_ALC_OVF_DISABLE		 				0x00

#define MAX30105_INT_PROX_INT_MASK  						(uint8_t)(~0b00010000)
#define MAX30105_INT_PROX_INT_ENABLE  						0x10
#define MAX30105_INT_PROX_INT_DISABLE		 				0x00

#define MAX30105_INT_DIE_TEMP_RDY_MASK 						(uint8_t)(~0b00000010)
#define MAX30105_INT_DIE_TEMP_RDY_ENABLE  					0x02
#define MAX30105_INT_DIE_TEMP_RDY_DISABLE  					0x00

#define MAX30105_SAMPLEAVG_MASK 							(uint8_t)(~0b11100000)
#define MAX30105_SAMPLEAVG_1  								0x00
#define MAX30105_SAMPLEAVG_2  								0x20
#define MAX30105_SAMPLEAVG_4  								0x40
#define MAX30105_SAMPLEAVG_8  								0x60
#define MAX30105_SAMPLEAVG_16  								0x80
#define MAX30105_SAMPLEAVG_32 							 	0xA0

#define MAX30105_ROLLOVER_MASK  							0xEF
#define MAX30105_ROLLOVER_ENABLE  							0x10
#define MAX30105_ROLLOVER_DISABLE  							0x00

#define MAX30105_A_FULL_MASK  								0xF0

// Mode configuration commands (page 19)
#define MAX30105_SHUTDOWN_MASK  							0x7F
#define MAX30105_SHUTDOWN  									0x80
#define MAX30105_WAKEUP  									0x00

#define MAX30105_RESET_MASK  								0xBF
#define MAX30105_RESET  									0x40

#define MAX30105_MODE_MASK  								0xF8
#define MAX30105_MODE_REDONLY  								0x02
#define MAX30105_MODE_REDIRONLY 	 						0x03
#define MAX30105_MODE_MULTILED 		 						0x07

// Particle sensing configuration commands (pgs 19-20)
#define MAX30105_ADCRANGE_MASK  							0x9F
#define MAX30105_ADCRANGE_2048  							0x00
#define MAX30105_ADCRANGE_4096 						 		0x20
#define MAX30105_ADCRANGE_8192  							0x40
#define MAX30105_ADCRANGE_16384  							0x60

#define MAX30105_SAMPLERATE_MASK 							0xE3
#define MAX30105_SAMPLERATE_50  							0x00
#define MAX30105_SAMPLERATE_100  							0x04
#define MAX30105_SAMPLERATE_200  							0x08
#define MAX30105_SAMPLERATE_400  							0x0C
#define MAX30105_SAMPLERATE_800  							0x10
#define MAX30105_SAMPLERATE_1000  							0x14
#define MAX30105_SAMPLERATE_1600  							0x18
#define MAX30105_SAMPLERATE_3200  							0x1C

#define MAX30105_PULSEWIDTH_MASK  							0xFC
#define MAX30105_PULSEWIDTH_69  							0x00
#define MAX30105_PULSEWIDTH_118  							0x01
#define MAX30105_PULSEWIDTH_215  							0x02
#define MAX30105_PULSEWIDTH_411  							0x03

//Multi-LED Mode configuration (pg 22)
#define MAX30105_SLOT1_MASK  								0xF8
#define MAX30105_SLOT2_MASK  								0x8F
#define MAX30105_SLOT3_MASK  								0xF8
#define MAX30105_SLOT4_MASK  								0x8F

#define SLOT_NONE  											0x00
#define SLOT_RED_LED  										0x01
#define SLOT_IR_LED  										0x02
#define SLOT_GREEN_LED  									0x03
#define SLOT_NONE_PILOT 									0x04
#define SLOT_RED_PILOT							 			0x05
#define SLOT_IR_PILOT 										0x06
#define SLOT_GREEN_PILOT 							 		0x07

#define MAX_30105_EXPECTEDPARTID  							0x15
#define true												1
#define false												0

#endif
