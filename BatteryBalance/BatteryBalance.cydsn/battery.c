/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "project.h"
#include "io.h"

uint8 MICHAEL_LOW = 0;
uint8 KEN_LOW = 0;
    
uint32 I2C_RW_Address(uint32 baseAddress, char read)
{   
    return (read > 1 || read < 0) ? 0 :
        (baseAddress << 1 | read);
}

uint8 DataPeriphRead(uint32 address, uint8* store)
{
    while (
        I2C_Brain_I2CMasterStatus() == I2C_Brain_I2C_MSTAT_XFER_INP ||
        I2C_DataPeriph_I2CSlaveStatus() == I2C_DataPeriph_I2C_SSTAT_RD_BUSY
    ) {}
    uint32 status = I2C_Brain_I2CMasterSendStart(address, I2C_Brain_I2C_READ_XFER_MODE,
            I2C_TIMEOUT);
    while (status != I2C_Brain_I2C_MSTR_NOT_READY) {
        status = I2C_Brain_I2CMasterSendStart(address, I2C_Brain_I2C_READ_XFER_MODE,
            I2C_TIMEOUT);
    }
    if (status == I2C_Brain_I2C_MSTR_NO_ERROR) {
        I2C_Brain_I2CMasterReadByte(I2C_Brain_I2C_NAK_DATA, store, I2C_TIMEOUT);
        return SUCCESS;
    } else {
        return FAIL;
    }
}

uint8 JettySend(uint32 address, uint32 data)
{
    while (
        I2C_Brain_I2CMasterStatus() == I2C_Brain_I2C_MSTAT_XFER_INP ||
        Jetty_I2CSlaveStatus() == Jetty_I2C_SSTAT_WR_BUSY
    ) {}
    uint32 status = I2C_Brain_I2CMasterSendStart(address, I2C_Brain_I2C_WRITE_XFER_MODE,
            I2C_TIMEOUT);
    while (status != I2C_Brain_I2C_MSTR_NOT_READY) {
        status = I2C_Brain_I2CMasterSendStart(address, I2C_Brain_I2C_WRITE_XFER_MODE,
            I2C_TIMEOUT);
    }
    if (status == I2C_Brain_I2C_MSTR_NO_ERROR) {
        I2C_Brain_I2CMasterWriteByte(data, I2C_TIMEOUT);
        return SUCCESS;
    } else {
        return FAIL;
    }
}

uint32 convertValue(uint8 adc_value) {
    return (uint32)adc_value;   
}


void BatteryBalanceInit(void)
{
    I2C_Brain_Start();
    
    I2C_DataPeriph_Start();
    I2C_DataPeriph_I2CSlaveSetAddress(DATA_ADDRESS);
    
    Jetty_Start();
    Jetty_I2CSlaveSetAddress(JETTY);
}

void BatteryBalance(uint8* battery, uint8* value, float32* threshold)
{
    float32 newVal = convertValue(*value);
    
    if (newVal < 0.1) {
        if (*battery == MICHAEL) {
            MICHAEL_LOW = 1;
            // update LED
        } else {
            KEN_LOW = 1;
            // update LED
        }
    }
    
    if (MICHAEL_LOW && *battery == MICHAEL && newVal >= 0.1) {
        MICHAEL_LOW = 0;
        // update LED
    } else if (KEN_LOW && *battery == KEN && newVal >= 0.1) {
        KEN_LOW = 0;
        // update LED
    }
    
    if (MICHAEL_LOW && !KEN_LOW) {
        // switch to KEN
        *battery = KEN;
    } else if (KEN_LOW && !MICHAEL_LOW) {
        // switch to MICHAEL
        *battery = MICHAEL;
    } else if (!(KEN_LOW || MICHAEL_LOW)) {
        if (newVal < *threshold - 0.1) {
            // switch battery
            if (*battery == MICHAEL) {
                *battery = KEN;
            } else {
                *battery = MICHAEL;
            }
            *threshold = newVal;
        } 
    }
}
*/

/* [] END OF FILE */
