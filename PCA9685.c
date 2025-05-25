/*
 * File:   PCA9685.c
 * Author: micha
 *
 * Created on 1 mai 2021, 10:32
 */


#include "PCA9685.h"




/**
 * \fn PCA9685_Init
 * \brief Fonction de configuration du PCA9685.
 *
 * \param    void
 * \return   void
 */
void PCA9685_Init()
{
    PCA9685_Reset();
    PCA9685_Set_PWM_Frequency(1000);	
}



/**
 * \fn Reset_PCA9685()
 * \brief Fonction de reset du PCA9685.
 *
 * \param    void
 * \return   void
 */
void PCA9685_Reset()
{
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_MODE_1, 0x00); // Normal mode
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_MODE_2, 0x04); // totem pole (default)
}


//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685_Set_PWM_Frequency(int freq)
{
    uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;

    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_MODE_1, 0x10); //sleep
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_PRE_SCALE, prescale_val); // multiplyer for PWM frequency
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_MODE_1, 0x80); //restart
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_MODE_2, 0x04); //totem pole (default)
}




/**
 * \fn PCA9685_Set_PWM_Duty()
 * \brief Fonction  de set de la duty cycle d'un PWM.
 *
 * \param    void
 * \return   void
 */
void PCA9685_Set_PWM_Duty(uint8_t led, int value) 
{
    PCA9685_Set_PWM_Duty_REG(led, 0, value);
}



/**
 * \fn PCA9685_Set_PWM_Duty_REG()
 * \brief Fonction  de set de la duty cycle d'un PWM.
 *
 * \param    ch : numéro du PWM
 * \param    on_value : valeur de la duty cycle
 * \param    off_value : valeur de la duty cycle
 * \return   void
 */
void PCA9685_Set_PWM_Duty_REG(uint8_t ch, int on_value, int off_value)
{
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_ON_L + PCA9685_CH_LED_MULTIPLYER * (ch), on_value & 0xFF); 
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_ON_H + PCA9685_CH_LED_MULTIPLYER * (ch), on_value >> 8); 

    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_OFF_L + PCA9685_CH_LED_MULTIPLYER * (ch), off_value & 0xFF); 
    PCA9685_Write(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_OFF_H + PCA9685_CH_LED_MULTIPLYER * (ch), off_value >> 8); 
}



/**
 * \fn PCA9685_Get_PWM()
 * \brief Fonction de get de la duty cycle d'un PWM.
 *
 * \param    ch : numéro du PWM
 * \return   int : valeur de la duty cycle
 */
int PCA9685_Get_PWM(uint8_t ch)
{
    int ledval = 0;
    uint8_t temp = 0;
    uint8_t temp2 = 0;

    temp = PCA9685_Read(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_ON_L + PCA9685_CH_LED_MULTIPLYER * (ch));
    temp2 = PCA9685_Read(PCA9685_SLAVE_ADDR, PCA9685_REG_ADDR_LED0_ON_H + PCA9685_CH_LED_MULTIPLYER * (ch));
    ledval = temp | (temp2 << 8);

    return ledval;
}





void PCA9685_Write(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t Data) 
{
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




uint8_t PCA9685_Read(uint16_t SlaveDeviceAddress, uint8_t RegisterAddress, uint8_t *Data) 
{
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