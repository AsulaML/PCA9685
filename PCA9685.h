#ifndef _PCA9685_H
#define _PCA9685_H

#include "I2C2.h"


#define PCA9685_SLAVE_ADDR 0b1000000

// Register Definitions

#define PCA9685_REG_ADDR_MODE_1 0x00			// Mode  register  1
#define PCA9685_REG_ADDR_MODE_2 0x01			// Mode  register  2
#define PCA9685_REG_ADDR_SUBADR1 0x02		    // I2C-bus subaddress 1
#define PCA9685_REG_ADDR_SUBADR2 0x03		    // I2C-bus subaddress 2
#define PCA9685_REG_ADDR_SUBADR3 0x04		    // I2C-bus subaddress 3
#define PCA9685_REG_ADDR_ALLCALLADR 0x05        // LED All Call I2C-bus address
#define PCA9685_REG_ADDR_LED0 0x6			    // LED0 start register
#define PCA9685_REG_ADDR_LED0_ON_L 0x6		    // LED0 output and brightness control byte 0
#define PCA9685_REG_ADDR_LED0_ON_H 0x7		    // LED0 output and brightness control byte 1
#define PCA9685_REG_ADDR_LED0_OFF_L 0x8		    // LED0 output and brightness control byte 2
#define PCA9685_REG_ADDR_LED0_OFF_H 0x9		    // LED0 output and brightness control byte 3
#define PCA9685_REG_ADDR_ALLLED_ON_L 0xFA       // Load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define PCA9685_REG_ADDR_ALLLED_ON_H 0xFB	    // Load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define PCA9685_REG_ADDR_ALLLED_OFF_L 0xFC	    // Load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define PCA9685_REG_ADDR_ALLLED_OFF_H 0xFD	    // Load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PCA9685_REG_ADDR_PRE_SCALE 0xFE		    // Prescaler for output frequency

#define CLOCK_FREQ 25000000.0                   //25MHz default osc clock
#define PCA9685_CH_LED_MULTIPLYER 4	            // For the other 15 channels


void PCA9685_Init();
void PCA9685_Reset();
void PCA9685_Set_PWM_Frequency(int freq);
void PCA9685_Set_PWM_Duty(uint8_t led, int value);
void PCA9685_Set_PWM_Duty_REG(uint8_t led, int on_value, int off_value);
int PCA9685_Get_PWM(uint8_t led);


void PCA9685_Write(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t Data);
uint8_t PCA9685_Read(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t *Data);


#endif