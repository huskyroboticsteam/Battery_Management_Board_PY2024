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

#ifndef __IO__
#define __IO__

#define SUCCESS (uint8)1
#define FAIL (uint8)0
    
#define I2C_TIMEOUT (uint32)500
#define DATA_ADDRESS (uint32)0x8
#define JETTY (uint32)0xC
#define MICHAEL (uint8)0x1
#define KEN (uint8)0x0
    
uint32 I2C_RW_Address(uint32 address, char read);

uint32 convertValue(uint8 adc_value);

void BatteryBalanceInit(void);

uint8 DataPeriphRead(uint32 address, uint8* store);

uint8 JettySend(uint32 address, uint32 data);

// void BatteryBalance(uint8* battery, uint8* value, float32* threshold);
    
#endif // __IO__

/* [] END OF FILE */
