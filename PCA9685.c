/*
 * File:   PCA9685.c
 * Author: micha
 *
 * Created on 1 mai 2021, 10:32
 */


#include <xc.h>
#include "PCA9685.h"



void Init_PCA9685()
{
	Reset_PCA9685();
	setPWMFrequency(1000);	
}



/**
 * \fn Reset_PCA9685()
 * \brief Fonction de reset du PCA9685.
 *
 * \param    void
 * \return   void
 */
void Reset_PCA9685()
{
	PCA9685_Write(PCA9685_ADDR, MODE1, 0x00); // Normal mode
	PCA9685_Write(PCA9685_ADDR, MODE2, 0x04); // totem pole (default)
}


//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void setPWMFrequency(int freq)
{
	uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
	PCA9685_Write(PCA9685_ADDR, MODE1, 0x10); //sleep
	PCA9685_Write(PCA9685_ADDR, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
	PCA9685_Write(PCA9685_ADDR, MODE1, 0x80); //restart
	PCA9685_Write(PCA9685_ADDR, MODE2, 0x04); //totem pole (default)
}




//! PWM a single channel
/*!
 \param led channel (0-15) to set PWM value for
 \param value 0-4095 value for PWM
 */
void setPWM_PCA9685(uint8_t led, int value) 
{
		setPWM(led, 0, value);
}



//! PWM a single channel with custom on time
/*!
 \param led channel (0-15) to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void setPWM(uint8_t led, int on_value, int off_value)
{
		PCA9685_Write(PCA9685_ADDR, LED0_ON_L + LED_MULTIPLYER * (led), on_value & 0xFF); 
		PCA9685_Write(PCA9685_ADDR, LED0_ON_H + LED_MULTIPLYER * (led), on_value >> 8); 
		
		PCA9685_Write(PCA9685_ADDR, LED0_OFF_L + LED_MULTIPLYER * (led), off_value & 0xFF); 
		PCA9685_Write(PCA9685_ADDR, LED0_OFF_H + LED_MULTIPLYER * (led), off_value >> 8); 
}



//! Get current PWM value
/*!
 \param led channel (0-15) to get PWM value from
 */
int getPWM(uint8_t led)
{
	int ledval = 0;
	uint8_t temp = 0;
	
	Read_PCA9685(PCA9685_ADDR, LED0_OFF_H + LED_MULTIPLYER * (led) , &temp);
	ledval = temp & 0xf;
	ledval <<= 8;
	Read_PCA9685(PCA9685_ADDR, LED0_OFF_L + LED_MULTIPLYER * (led) , &temp);
	ledval += temp;
	
	return ledval;
}




void PCA9685_Write(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t Data) {
    
   
    uint8_t         writeBuffer[5];
    uint16_t        timeOut, slaveTimeOut;

    // this initial value is important
    I2C2_MESSAGE_STATUS status = I2C2_MESSAGE_PENDING;

    // build the write buffer first
    // starting 8-bit register address
    writeBuffer[0] = RegisterAddress;

    // data to be written
    writeBuffer[1] = (uint8_t)(Data);

    // Now it is possible that the slave device will be slow.
    // As a work around on these slaves, the application can
    // retry sending the transaction
    timeOut = 0;
    slaveTimeOut = 0;

    while(status != I2C2_MESSAGE_FAIL)
    {
        // write
        I2C2_MasterWrite(writeBuffer,
                         2,
                         SlaveDeviceAddress,
                         &status);

        // wait for the message to be sent or status has changed.
        while(status == I2C2_MESSAGE_PENDING)
        {
            // add some delay here 2*50 = 100us
            delay_ms(200);
            
            // timeout checking
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        } 
        
        if ((slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT) || 
            (status == I2C2_MESSAGE_COMPLETE))
            break;

        // if status is  I2C2_MESSAGE_ADDRESS_NO_ACK,
        //               or I2C2_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
            break;
        else
            timeOut++;
    }
}




uint8_t Read_PCA9685(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t *Data) {
    
    
    I2C2_MESSAGE_STATUS status;
    I2C2_TRANSACTION_REQUEST_BLOCK readTRB[2];
    uint8_t     writeBuffer[3];
    uint8_t     readBuffer[4];
    uint16_t    timeOut, slaveTimeOut;

    // this initial value is important
    status = I2C2_MESSAGE_PENDING;

    // build the write buffer first
    // starting 8-bit register address
    writeBuffer[0] = RegisterAddress;

    // we need to create the TRBs for a random read sequence to the EEPROM
    // Build TRB for sending address
    I2C2_MasterWriteTRBBuild( &readTRB[0],
                              writeBuffer,
                              1,
                              SlaveDeviceAddress);
    
    
    // Build TRB for receiving data
    I2C2_MasterReadTRBBuild( &readTRB[1],
                             readBuffer,
                             1,
                             SlaveDeviceAddress);

    timeOut = 0;
    slaveTimeOut = 0;

    while(status != I2C2_MESSAGE_FAIL)
    {
        // now send the transactions
        I2C2_MasterTRBInsert(2, readTRB, &status);

        // wait for the message to be sent or status has changed.
        while(status == I2C2_MESSAGE_PENDING)
        {
            // add some delay here
            delay_ms(200);
            
            
            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
                return (0);
            else
                slaveTimeOut++;
        }

        if (status == I2C2_MESSAGE_COMPLETE)
            break;

        // if status is  I2C2_MESSAGE_ADDRESS_NO_ACK,
        //               or I2C2_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
            return (0);
        else
            timeOut++;

    }
    
    uint8_t value = readBuffer[0];
    *Data = value;
    return (1);
}   